from sensors import sensor
from modules.qtimshow import imshow


class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor

    def execute(self):
        # Add your code here
        img = self.sensor.getImage()
        self.sensor.takeoff()
        imshow('first-person view', img)
