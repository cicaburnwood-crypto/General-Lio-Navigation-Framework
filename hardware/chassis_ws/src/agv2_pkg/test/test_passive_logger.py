"""No ROS/network/CAN access: validate passive logging and raw capture replay."""
import inspect
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from faultlog.can_helper import Queries, QUERY, run

class PassiveLogger(unittest.TestCase):
    def test_runtime_has_no_send_path(self):
        source = inspect.getsource(run)
        self.assertNotIn('raw.send', source)
        self.assertNotIn('enabled,', source)
        self.assertIn('queries.step(now, False, None, emit)', source)

    def test_tracks_driver_request_and_reply_without_sending(self):
        q = Queries([321, 322]); events = []
        emit = lambda *args: events.append(args)
        tx = dict(id=321, origin='tx_observed', data_hex=QUERY.hex(), kernel_time_ns=100)
        q.observe_query(tx, 1., emit)
        self.assertIn(321, q.pending)
        rx = dict(id=321, origin='bus_rx', data_hex='9a21800902000000', error=False, rtr=False, kernel_time_ns=102)
        q.receive(rx, 1.002, emit)
        self.assertNotIn(321, q.pending)
        q.step(2., False, None, emit)
        self.assertEqual(q.failures[321], 0)
        self.assertEqual(events[-1][2]['status'], 'reply')

    def test_reordered_capture_cannot_make_false_timeout(self):
        q = Queries([321, 322]); events = []
        emit = lambda *args: events.append(args)
        q.receive(dict(id=321, origin='bus_rx', data_hex='9a21800902000000', error=False, rtr=False, kernel_time_ns=102), 1.002, emit)
        q.observe_query(dict(id=321, origin='tx_observed', data_hex=QUERY.hex(), kernel_time_ns=100), 1.003, emit)
        self.assertNotIn(321, q.pending)

if __name__ == '__main__': unittest.main()
