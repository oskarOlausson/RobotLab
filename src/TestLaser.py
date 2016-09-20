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
        self.assertTrue(RobotMove.robotCanSee(y, goalx, goaly, 0, x))

        goalx, goaly = 2.673, 0.145
        self.assertTrue(RobotMove.robotCanSee(y, goalx, goaly, 0, x))

        goalx, goaly = 5, 5
        self.failIf(RobotMove.robotCanSee(y, goalx, goaly, 0, x))

    def test_robotCanBe(self):
        x, y = 0, 0
        goalx, goaly = 1, 1
        self.assertTrue(RobotMove.robotCanBe(y, goalx, goaly, 0, x))

        goalx, goaly = 2.673, 0.145
        self.failIf(RobotMove.robotCanBe(y, goalx, goaly, 0, x))

        goalx, goaly = 5, 5
        self.failIf(RobotMove.robotCanBe(y, goalx, goaly, 0, x))



if __name__ == '__main__':
    unittest.main()
