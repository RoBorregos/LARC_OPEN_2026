"""Readiness must be earned by reports, never by process launch/grace."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch, Mock

path = Path(__file__).resolve().parents[1] / 'dispatcher' / 'dispatcher.py'
spec = importlib.util.spec_from_file_location('dispatcher_readiness_test', path)
d = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'serial': types.ModuleType('serial')}):
    spec.loader.exec_module(d)

class ReadinessTests(unittest.TestCase):
    def test_grace_does_not_mean_ready(self):
        s=d.VisionState()
        with patch.object(d, 'now_ms', return_value=1000):
            s.note_launched(['intake', 'separator'])
            self.assertFalse(s.ready(['intake', 'separator']))
            self.assertEqual(s.beans_command()[-1], set())
            s.feed('intake', '00')
            self.assertFalse(s.ready(['intake', 'separator']))
            s.feed('separator', 'FD:00:00')
            self.assertTrue(s.ready(['intake', 'separator']))
        with patch.object(d, 'now_ms', return_value=1501):
            self.assertFalse(s.ready(['intake', 'separator']))
        s.reset()
        self.assertFalse(s.ready(['intake']))

    def test_slow_start_stays_unready_without_fault_until_deadline(self):
        s=d.VisionState()
        with patch.object(d, 'STARTUP_GRACE_SEC', 120), patch.object(d, 'now_ms', return_value=1000):
            s.note_launched(['intake','separator'])
        with patch.object(d, 'now_ms', return_value=61000):
            self.assertFalse(s.ready(['intake','separator']))
            self.assertEqual(s.beans_command()[-1],set())
        with patch.object(d, 'now_ms', return_value=122000):
            self.assertEqual(s.beans_command()[-1],{'intake','separator'})

    def test_benefits_neutral_is_valid_first_frame(self):
        s=d.VisionState()
        s.note_launched(['benefits'])
        self.assertFalse(s.ready(['benefits']))
        s.feed('benefits', 'FE:00')
        self.assertTrue(s.ready(['benefits']))
        self.assertEqual(s.benefits_command()[:2], (False,False))

    def test_status_waits_for_reports_and_clears_on_stall(self):
        x=d.Dispatcher(Mock())
        x.link=Mock()
        x.runner=Mock(phase='beans')
        x.runner.alive.return_value=['intake','separator']
        with patch.object(d, 'now_ms', return_value=1000):
            x.state.note_launched(['intake','separator'])
            x.push_command()
            x.link.set_beans_running.assert_called_with(False)
            x.state.feed('intake','01'); x.state.feed('separator','FD:01:00')
            x.push_command()
            x.link.set_beans_running.assert_called_with(True)
        with patch.object(d, 'now_ms', return_value=2000):
            x.push_command()
            x.link.set_beans_running.assert_called_with(False)
            x.link.set_beans.assert_called_with(False,False,d.vp.SEP_NEUTRAL)

    def test_new_phase_clears_previous_fault_and_log_state(self):
        x=d.Dispatcher(Mock()); x.link=Mock(); x.runner=Mock(phase=None)
        x.runner.start.return_value=True
        x._stale_reported={'intake','separator'}
        x._last_reported='old'
        x.enter('beans')
        self.assertEqual(x._stale_reported,set())
        self.assertIsNone(x._last_reported)
        x.link.set_beans_running.assert_called_once_with(False)
        for bit in d.FAULT_BIT.values():
            x.link.set_fault.assert_any_call(bit,False)

if __name__=='__main__': unittest.main()
