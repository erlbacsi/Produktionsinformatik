import time

class TimeMeasurement:
    def __init__(self):
        self.data = open("TimeLogger.txt", "a")    #Textdatei die erzeugt wird
        self.starting = 0
        self.ending = 0

    def start(self):
        self.starting = time.perf_counter()        #Start der Zeitzählung

    def end(self, description):
        self.ending = time.perf_counter()          #Ende der Zeitzählung
        self.data.write("Ausfuehrungszeit von {}: {:.2f} Sekunden\n".format(description, self.ending - self.starting))

    def closeData(self):
        self.data.close()
