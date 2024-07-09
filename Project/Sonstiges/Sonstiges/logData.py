import logging

logging.basicConfig(filename="Test.log", level=logging.DEBUG)
logging.info("Fehler ist aufgetreten")

'''
#Start logger
logger = logging.getLogger("logfile.log")
#Setzen der Startebene für die Ausgabe, ab DEBUG level wird alles ausgegeben
logger.setLevel("DEBUG")
#Überprüfen ob bereits handler existieren
if not logger.handlers:
   #console = logging.StreamHandler()
   #console.setLevel('DEBUG')
   file = logging.FileHandler(logger.name, mode='w', encoding='utf-8')
   file.setLevel('DEBUG')
   logger.addHandler(file)
   #logger.addHandler(console)

logger.info("STL-Datei: %s wird eingelesen...",pfad)
logging.shutdown()



logging.basicConfig(filename="Test.log", level=logging.INFO)
logging.warning("This is a warning!")
logging.info("This is a Info")
'''