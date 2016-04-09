from sensors import sensor
from modules.qtimshow import imshow

from varribas.main_varribas import EntryPoint


class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.varribas = EntryPoint(sensor)

    def execute(self):
        # Add your code here
        self.varribas.execute()
        # img = self.sensor.getImage()
        # imshow('first-person view', img)
