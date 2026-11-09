import base64
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT/'dashboard'))
import cv2
import numpy as np
import app
from aubo_huawei_bridge.camera import CameraBuffer


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.now=[10.0]
        self.frames=CameraBuffer(clock=lambda:self.now[0])
        env={'AUBO_DASHBOARD_MODE':'ros','DASHBOARD_PASSWORD':'private-test-password-long',
             'DASHBOARD_SESSION_KEY':'t'*40,'DASHBOARD_ENABLE_COMMANDS':'false'}
        with patch.dict(os.environ,env):
            self.application=app.App(SimpleNamespace(camera_enabled=True,camera=self.frames.get))
        self.token=self.application.sign('login',{'exp':self.application.clock()+60})

    def image(self):
        return np.random.RandomState(5).randint(0,256,(1080,1920,3),dtype=np.uint8)

    def test_large_image_becomes_bounded_valid_jpeg_and_does_not_outlive_three_seconds(self):
        self.assertTrue(self.frames.update(self.image()))
        jpeg,age,sequence=self.frames.get()
        self.assertLessEqual(len(jpeg),32000)
        decoded=cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),cv2.IMREAD_COLOR)
        self.assertLessEqual(decoded.shape[1],640)
        self.assertLessEqual(decoded.shape[0],480)
        self.assertAlmostEqual(decoded.shape[1]/decoded.shape[0],1920/1080,delta=.03)
        self.assertEqual(sequence,1)
        self.assertFalse(self.frames.update(self.image()))
        self.now[0]+=1
        self.assertTrue(self.frames.update(self.image()))
        self.assertEqual(self.frames.get()[2],2)
        self.now[0]+=3.01
        self.assertIsNone(self.frames.get())

    def test_camera_requires_login_and_stale_frame_returns_503(self):
        self.frames.update(self.image())
        self.assertEqual(self.application.route('GET','/api/camera')[0],401)
        headers={'Authorization':'Bearer '+self.token}
        status,response_headers,body=self.application.route('GET','/api/camera',headers)
        self.assertEqual(status,200)
        self.assertEqual(response_headers['Content-Type'],'image/jpeg')
        self.assertEqual(response_headers['Cache-Control'],'no-store')
        self.assertTrue(body.startswith(b'\xff\xd8'))
        self.assertIsNotNone(cv2.imdecode(np.frombuffer(body,dtype=np.uint8),cv2.IMREAD_COLOR))
        with patch.object(app,'_application',self.application):
            event={'httpMethod':'GET','path':'/api/camera','headers':headers}
            response=app.handler(event,None)
            self.assertTrue(response['isBase64Encoded'])
            self.assertEqual(base64.b64decode(response['body']),body)
        self.now[0]+=4
        self.assertEqual(self.application.route('GET','/api/camera',headers)[0],503)


if __name__=='__main__':
    unittest.main()
