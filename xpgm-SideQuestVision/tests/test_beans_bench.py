from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'link'))
from beans_bench import BeansBench
import vision_protocol as vp

class Link:
    device='fake'; healthy=True; error=None
    def __init__(self): self.requests=[];self.messages=[];self.phase='idle';self.command=None
    def set_ready(self,value): pass
    def set_idle(self): self.phase='idle';self.command=(False,False,0)
    def set_beans_running(self,value): self.running=value
    def set_beans(self,*command): self.phase='beans';self.command=command
    def take_requests(self): result,self.requests=self.requests,[];return result
    def take_messages(self): result,self.messages=self.messages,[];return result
    def close(self): self.closed=True
    def halt(self): self.phase='halt'

class BenchTests(unittest.TestCase):
    def setUp(self):
        self.now=10.;self.link=Link()
        self.bench=BeansBench(link=self.link,start_thread=False,clock=lambda:self.now)
    def start(self): self.link.requests=[vp.CMD_START_BEANS];self.bench.tick()
    def test_waits_for_hold_sketch_request(self):
        self.bench.publish('intake',{'right':True,'left':True},self.now)
        self.bench.tick();self.assertEqual(self.link.phase,'idle')
        self.start();self.assertEqual(self.link.command,(True,True,0))
    def test_camera_or_inference_stall_expires_command(self):
        self.start()
        self.bench.publish('intake',{'right':True},self.now)
        self.bench.tick();self.assertTrue(self.link.command[0])
        self.now+=.401;self.bench.tick()
        self.assertEqual(self.link.command,(False,False,0))
    def test_old_frame_cannot_rearm(self):
        self.start();self.bench.publish('intake',{'left':True},self.now-1)
        self.bench.tick();self.assertFalse(self.link.command[1])
    def test_error_clears_detection(self):
        self.start();self.bench.publish('intake',{'right':True,'error':'invalid ROI'},self.now)
        self.bench.tick();self.assertFalse(self.link.command[0])
    def test_reprocessed_frame_does_not_extend_hold(self):
        self.start();stamp=self.now
        self.bench.publish('separator',{'warm':True,'cool':False},stamp)
        self.bench.tick();self.assertEqual(self.link.command[2],vp.SEP_LEFT)
        self.now+=.2
        self.bench.publish('separator',{'warm':True,'cool':False},stamp);self.bench.tick()
        self.now+=.06;self.bench.tick();self.assertEqual(self.link.command[2],0)
    def test_new_frame_refreshes_hold(self):
        self.start();self.bench.publish('separator',{'cool':True},self.now);self.bench.tick()
        self.now+=.2;self.bench.publish('separator',{'cool':True},self.now);self.bench.tick()
        self.now+=.1;self.bench.tick();self.assertEqual(self.link.command[2],vp.SEP_RIGHT)
    def test_stop_and_unsupported_phase_go_idle(self):
        for request in (vp.CMD_STOP,vp.CMD_START_BENEFITS):
            self.start();self.link.requests=[request];self.bench.tick()
            self.assertEqual(self.link.phase,'idle')
    def test_teensy_feedback_visible(self):
        self.link.messages=['[apply] intake upper -> DEPLOY']
        self.bench.tick();self.assertIn('DEPLOY',self.bench.snapshot()['messages'][0])
    def test_close_parks(self):
        self.start();self.bench.close()
        self.assertEqual(self.link.phase,'idle');self.assertTrue(self.link.closed)
if __name__=='__main__': unittest.main()
