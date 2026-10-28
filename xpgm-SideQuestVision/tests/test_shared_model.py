"""Lifecycle and safety regressions without cameras, CUDA, or robot hardware."""
import importlib.util
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from model_runtime.shared import SharedModelWorker
from model_runtime import core, capture
spec = importlib.util.spec_from_file_location('shared_dispatcher_test', ROOT/'dispatcher/dispatcher.py')
d = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'serial': types.ModuleType('serial')}):
    spec.loader.exec_module(d)


class SharedTests(unittest.TestCase):
    def test_stop_and_reentry_discard_old_inference_and_error_invalidates(self):
        state = d.VisionState()
        worker = SharedModelWorker(['intake', 'separator'], state.feed, state.invalidate, Mock())
        worker.max_age = .4
        worker.activate()
        old = worker.generation
        worker.deliver(old, 'intake', {'message': 'VISION:01'}, time.monotonic())
        self.assertTrue(state.ready(['intake']))
        worker.deactivate()
        state.reset()
        worker.deliver(old, 'intake', {'message': 'VISION:01'}, time.monotonic())
        self.assertFalse(state.ready(['intake']))
        worker.activate()
        worker.deliver(old, 'intake', {'message': 'VISION:01'}, time.monotonic())
        worker.deliver(worker.generation, 'intake', {'message': 'VISION:01'}, worker.activated_at-.01)
        self.assertFalse(state.ready(['intake']))
        worker.deliver(worker.generation, 'intake', {'message': 'VISION:01'}, time.monotonic())
        worker.deliver(worker.generation, 'separator', {'message': 'VISION:FD:01:00'}, time.monotonic())
        self.assertTrue(state.ready(['intake','separator']))
        worker.deliver(worker.generation, 'separator', {'error': 'camera lost'}, time.monotonic())
        self.assertFalse(state.ready(['intake','separator']))
        self.assertEqual(state.beans_command()[2], d.vp.SEP_NEUTRAL)

    def test_expired_result_cannot_refresh_readiness(self):
        publish, invalidate = Mock(), Mock()
        worker = SharedModelWorker(['intake'], publish, invalidate, Mock())
        worker.max_age = .4
        worker.activate()
        stamp = worker.activated_at
        with patch('model_runtime.shared.time.monotonic', return_value=stamp+.5):
            worker.deliver(worker.generation, 'intake', {'message':'VISION:01'}, stamp)
        publish.assert_not_called()
        invalidate.assert_called_once_with('intake')

    def test_one_detector_survives_idle_and_reentry_with_sequential_calls(self):
        warmed, blocked, release, published, closed_camera = [threading.Event() for _ in range(5)]
        calls = []
        def infer(frame, role, cfg, stamp):
            calls.append((role, threading.get_ident()))
            if len(calls) == 1:
                blocked.set()
                if not release.wait(3):
                    raise RuntimeError('test release timed out')
            return {'message': 'VISION:00' if role=='intake' else 'VISION:FD:00:00',
                    'inference_ms': 1, 'frame_age_ms': 1}, None
        detector = Mock()
        detector.warmup.side_effect = lambda roles: warmed.set()
        detector.run.side_effect = infer
        class Camera:
            def __init__(self, role): self.seq = 0
            def latest(self):
                self.seq += 1
                return object(), time.monotonic(), self.seq, None
            def close(self): closed_camera.set()
        publications = []
        def publish(role, payload):
            publications.append(role)
            if set(publications) == {'intake', 'separator'}: published.set()
        worker = SharedModelWorker(['intake','separator'], publish, Mock(), Mock())
        with patch.object(core, 'settings', return_value={'max_frame_age_ms':400}), \
             patch.object(core, 'Detector', return_value=detector) as factory, \
             patch.object(core, 'load_roi', return_value={}), patch.object(capture, 'Camera', Camera):
            try:
                worker.start()
                self.assertTrue(warmed.wait(2))
                worker.activate()
                self.assertTrue(blocked.wait(2))
                worker.deactivate() # must return while inference remains blocked
                self.assertEqual(publications, [])
                release.set()
                self.assertTrue(closed_camera.wait(2))
                self.assertEqual(publications, [])
                worker.activate()
                self.assertTrue(published.wait(2))
            finally:
                release.set()
                worker.close()
            factory.assert_called_once()
            detector.warmup.assert_called_once()
            self.assertEqual(len({ident for _, ident in calls}), 1)
            self.assertFalse(worker.alive())

    def test_model_load_failure_is_visible_without_reports(self):
        worker = SharedModelWorker(['intake'], Mock(), Mock(), Mock())
        with patch.object(core, 'settings', return_value={'max_frame_age_ms':400}), \
             patch.object(core, 'Detector', side_effect=RuntimeError('CUDA unavailable')):
            worker.start()
            worker.thread.join(2)
        self.assertFalse(worker.alive())
        self.assertEqual(worker.error, 'CUDA unavailable')
        worker.publish.assert_not_called()

    def test_dispatcher_never_spawns_per_camera_models_and_retains_worker(self):
        state = d.VisionState()
        with patch.object(d, 'SharedModelWorker') as factory, patch.object(d.subprocess, 'Popen') as popen:
            factory.return_value.alive.return_value = True
            runner = d.ScriptRunner(state)
            runner.prepare()
            self.assertTrue(runner.start('beans'))
            self.assertEqual(set(runner.alive()), {'intake','separator'})
            popen.assert_not_called()
            runner.stop()
            factory.return_value.close.assert_not_called()
            self.assertFalse(state.ready(['intake','separator']))
            self.assertTrue(runner.start('beans'))
            factory.assert_called_once()
            popen.assert_not_called()
            runner.close()
            factory.return_value.close.assert_called_once()

    def test_mixed_backend_still_spawns_only_opencv_camera(self):
        scripts = dict(d.SCRIPTS, separator=ROOT/'separator/separator_vision.py')
        proc=Mock(stdout=[], pid=123)
        proc.poll.return_value=None
        with patch.dict(d.SCRIPTS, scripts), patch.object(d, 'SharedModelWorker') as factory, \
             patch.object(d.subprocess, 'Popen', return_value=proc) as popen:
            factory.return_value.alive.return_value=True
            runner=d.ScriptRunner(d.VisionState())
            self.assertTrue(runner.start('beans'))
            self.assertEqual(runner.model_roles, ['intake'])
            self.assertEqual(set(runner.alive()), {'intake','separator'})
            popen.assert_called_once()
            self.assertIn('separator_vision.py', popen.call_args.args[0][-1])
            runner.close()

    def test_opencv_only_does_not_create_a_model(self):
        scripts = dict(d.SCRIPTS, intake=ROOT/'main_vision/orin_vision.py',
                       separator=ROOT/'separator/separator_vision.py')
        with patch.dict(d.SCRIPTS, scripts), patch.object(d, 'SharedModelWorker') as factory:
            runner = d.ScriptRunner(d.VisionState())
            runner.prepare()
            self.assertIsNone(runner.model)
            factory.assert_not_called()
            runner.close()

    def test_benefits_transition_keeps_model_but_clears_beans(self):
        proc = Mock(stdout=[], pid=123)
        proc.poll.return_value = None
        with patch.object(d, 'SharedModelWorker') as factory, \
             patch.object(d.subprocess, 'Popen', return_value=proc) as popen:
            factory.return_value.alive.return_value = True
            state = d.VisionState()
            runner = d.ScriptRunner(state)
            runner.start('beans')
            state.feed('intake', '01')
            state.feed('separator', 'FD:01:00')
            runner.start('benefits')
            self.assertEqual(runner.alive(), ['benefits'])
            self.assertFalse(state.ready(['intake', 'separator']))
            factory.return_value.close.assert_not_called()
            popen.assert_called_once()
            runner.close()

    def test_shared_worker_death_faults_both_sources(self):
        with patch.object(d, 'SharedModelWorker') as factory:
            factory.return_value.alive.return_value = True
            runner = d.ScriptRunner(d.VisionState())
            runner.start('beans')
            runner.state.feed('intake', '01')
            runner.state.feed('separator', 'FD:01:00')
            factory.return_value.alive.return_value = False
            self.assertEqual(set(runner.reap()), {'intake', 'separator'})
            self.assertFalse(runner.state.ready(['intake', 'separator']))
            self.assertEqual(runner.alive(), [])
            runner.close()

    def test_shutdown_and_serial_loss_disable_worker(self):
        x=d.Dispatcher(Mock())
        x.link=Mock()
        x.runner=Mock()
        x.drop_link('disconnected')
        x.runner.stop.assert_called_once_with(quiet=True)
        self.assertIsNone(x.link)

if __name__ == '__main__': unittest.main()
