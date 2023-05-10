class Map:
    def __init__(self):
        self.x_range = (0, 60)
        self.y_range = (0, 20)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        r = 10
        obs_boundary = [
            [0, 0, 0.01, 20],
            [0, 20, 60, 0.01],
            [0.01, 0, 60, 0.01],
            [60, 0.01, 0.01, 20]
        ]
        return obs_boundary
    
    @staticmethod
    def obs_rectangle():
        rectangles = [
            [15,7.5,1.5,12.5],
            [25,0,1.5,12.5]
        ]
        return rectangles
    
    @staticmethod
    def obs_circle():
        circle = [40,11,5]
        return circle


