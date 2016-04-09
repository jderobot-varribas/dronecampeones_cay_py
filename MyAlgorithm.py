from sensors import sensor

from varribas.main_varribas import EntryPoint


class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.varribas = EntryPoint(sensor)

    def execute(self):
        # Add your code here
        self.varribas.execute()
