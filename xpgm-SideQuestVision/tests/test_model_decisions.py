"""Robot contract checks without cameras, torch, or servos."""
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from model_runtime.core import decide, neutral, read_json, regions

class Decisions(unittest.TestCase):
    def setUp(self):
        self.options=read_json(ROOT/'model_config.json')
        self.intake={'det_cy_l':286,'det_cy_r':379,'det_thick':97,'trigger_x':103,'trig_y2':376}
        self.separator={'roi_x':100,'roi_y':100,'roi_w':100,'roi_h':100}
    def ball(self,name,x,y,r=15):
        return {'name':name,'circle':[x,y,r],'box':[x-r,y-r,x+r,y+r],'confidence':.9}
    def intake_result(self,balls):
        return decide('intake',balls,self.intake,(376,1344),self.options,(1344,376))
    def separator_result(self,balls):
        return decide('separator',balls,self.separator,(480,640),self.options,(640,480))
    def test_right_is_upper_bit_zero(self):
        self.assertEqual(self.intake_result([self.ball('ball_black',672+379,200)])['message'],'VISION:01')
    def test_left_is_lower_bit_one(self):
        self.assertEqual(self.intake_result([self.ball('ball_blue',286,200)])['message'],'VISION:02')
    def test_both_and_green_rejection(self):
        balls=[self.ball('ball_red',286,200),self.ball('ball_yellow',1051,200)]
        self.assertEqual(self.intake_result(balls)['message'],'VISION:03')
        self.assertEqual(self.intake_result([self.ball('ball_green',286,200)])['message'],'VISION:00')
    def test_circle_touches_band_but_center_outside(self):
        self.assertTrue(self.intake_result([self.ball('ball_black',286,90,13)])['left'])
        self.assertFalse(self.intake_result([self.ball('ball_black',286,89,13)])['left'])
    def test_horizontal_rule_uses_center_not_edge(self):
        self.assertFalse(self.intake_result([self.ball('ball_black',180,200,30)])['left'])
    def test_separator_routes_and_ambiguity(self):
        for name,message in [('ball_red','VISION:FD:01:00'),('ball_yellow','VISION:FD:01:00'),('ball_blue','VISION:FD:00:01'),('ball_black','VISION:FD:00:01')]:
            self.assertEqual(self.separator_result([self.ball(name,150,150)])['message'],message)
        self.assertEqual(self.separator_result([self.ball('ball_red',150,150),self.ball('ball_blue',150,150)])['message'],'VISION:FD:01:01')
    def test_green_priority_preserved(self):
        self.assertEqual(self.separator_result([self.ball('ball_green',150,150),self.ball('ball_black',150,150)])['message'],'VISION:FD:01:00')
    def test_background_and_outside_are_neutral(self):
        self.assertEqual(self.separator_result([])['message'],'VISION:FD:00:00')
        self.assertEqual(self.separator_result([self.ball('ball_black',400,400)])['message'],'VISION:FD:00:00')
    def test_invalid_roi_fails_neutral(self):
        cfg={'roi_x':804,'roi_y':597,'roi_w':344,'roi_h':332}
        out=decide('separator',[self.ball('ball_black',639,479)],cfg,(480,640),self.options)
        self.assertEqual(out['message'],'VISION:FD:00:00')
        self.assertIn('outside',out['error'])
    def test_resolution_mismatch_fails_neutral(self):
        out=decide('intake',[],self.intake,(480,640),self.options,(1344,376))
        self.assertIsNotNone(out['error'])
        self.assertEqual(out['message'],'VISION:00')
    def test_unknown_class_not_collected(self):
        self.assertEqual(self.intake_result([self.ball('ball_orange',286,200)])['message'],'VISION:00')
if __name__=='__main__': unittest.main()
