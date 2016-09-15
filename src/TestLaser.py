import unittest
import RobotMove

"""
    This test requires that the simulation is running and the position
    of the robot be 0,0 with bearing 0
"""

class MyTestCase(unittest.TestCase):
    def test_robotCanSee(self):
        x, y = 0, 0
        goalx, goaly = 1, 1
        self.assertTrue(RobotMove.robotCanSee(x, y, goalx, goaly, 0))

        goalx, goaly = 2.673, 0.145
        self.assertTrue(RobotMove.robotCanSee(x, y, goalx, goaly, 0))

        goalx, goaly = 5, 5
        self.failIf(RobotMove.robotCanSee(x, y, goalx, goaly, 0))

    def test_robotCanBe(self):
        x, y = 0, 0
        goalx, goaly = 1, 1
        self.assertTrue(RobotMove.robotCanBe(x, y, goalx, goaly, 0))

        goalx, goaly = 2.673, 0.145
        self.failIf(RobotMove.robotCanBe(x, y, goalx, goaly, 0))

        goalx, goaly = 5, 5
        self.failIf(RobotMove.robotCanBe(x, y, goalx, goaly, 0))



if __name__ == '__main__':
    unittest.main()
