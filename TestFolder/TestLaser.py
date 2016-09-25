import unittest
import src.robotMove
from src.Postman import getLaser
import src.path

"""
    This test requires that the simulation is running and the position
    of the robot be 0,0 with bearing 0
"""

class MyTestCase(unittest.TestCase):
    def test_robotCanSee(self):
        laser=getLaser()
        x, y = 0, 0
        goalx, goaly = 1, 1
        self.assertTrue(src.robotMove.robotCanSee(x, y, goalx, goaly, 0, laser))

        goalx, goaly = 2.673, 0.145
        self.assertTrue(src.robotMove.robotCanSee(x, y, goalx, goaly, 0, laser))

        goalx, goaly = 5, 5
        self.failIf(src.robotMove.robotCanSee(x, y, goalx, goaly, 0, laser))

    def test_robotCanBe(self):
        x, y = 0, 0
        goalx, goaly = 1, 1
        laser = getLaser()
        self.assertTrue(src.robotMove.robotCanBe(x, y, goalx, goaly, 0, 0, laser))

        goalx, goaly = 1, 0
        self.assertTrue(src.robotMove.robotCanBe(x, y, goalx, goaly, 0, 0, laser))

        goalx, goaly = 5, 5
        self.failIf(src.robotMove.robotCanBe(x, y, goalx, goaly, 0, 0, laser))

        goalx, goaly = 4.97860813141, 3.56858062744
        self.failIf(src.robotMove.robotCanBe(x, y, goalx, goaly, 0, 0, laser))

        pathHandler = src.path.Path("testPath.json")
        for i in range(0,59):
            goalx, goaly = pathHandler.position(i)
            print goalx,goaly
            self.failUnless(src.robotMove.robotCanBe(x, y, goalx, goaly, 0, 0, laser))



if __name__ == '__main__':
    unittest.main()
