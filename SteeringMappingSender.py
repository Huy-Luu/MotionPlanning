
class SteeingMappingSender():
    @staticmethod
    def sendMapped(shandler, angle):
        if(angle == 30.0):
            shandler.send("a")
        if(angle == 20.0):
            shandler.send("j")
        if(angle == 10.0):
            shandler.send("h")
        if(angle == 0.0):
            shandler.send("g")
        if(angle == -10.0):
            shandler.send("l")
        if(angle == -20.0):
            shandler.send("k")
        if(angle == -30.0):
            shandler.send("d")