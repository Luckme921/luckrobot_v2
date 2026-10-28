import unittest
from robot_mode_gateway import Protocol
import json
class Test(unittest.TestCase):
 def setUp(self):
  self.calls=[];self.p=Protocol('test-token',lambda x:(self.calls.append(x) or True,'accepted'),lambda:{'state':'PAUSED'})
 def send(self,i='a',c='auto_mapping',token='test-token'):
  return self.p.handle(json.dumps(dict(id=i,command=c,token=token)))
 def test_dedup(self):
  self.assertTrue(self.send()['ok']);self.send();self.assertEqual(self.calls,['auto_mapping'])
 def test_conflict(self):
  self.send();self.assertFalse(self.send(c='pause')['ok']);self.assertEqual(len(self.calls),1)
 def test_auth(self):self.assertFalse(self.send(token='bad')['ok']);self.assertFalse(self.calls)
 def test_status(self):self.assertTrue(self.send(c='status')['ok']);self.assertFalse(self.calls)
 def test_unknown(self):self.assertFalse(self.send(c='cmd_vel')['ok']);self.assertFalse(self.calls)
 def test_invalid(self):
  for v in ['[]','null','bad','x'*4097]:self.assertFalse(self.p.handle(v)['ok'])
 def test_uncertain_not_repeated(self):
  def fail(x):self.calls.append(x);raise TimeoutError()
  self.p.dispatch=fail;self.send();self.send();self.assertEqual(len(self.calls),1)
if __name__=='__main__':unittest.main()
