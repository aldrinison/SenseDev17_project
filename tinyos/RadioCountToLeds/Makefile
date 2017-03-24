COMPONENT=RadioCountToLedsAppC
SENSORBOARD=mts400
CFLAGS += -I$(TOSDIR)/lib/printf
#CFLAGS += -I$(TOSDIR)/chips/atm128/i2c
CFLAGS += -DCC2420_DEF_CHANNEL=17
#CFLAGS += "-DCC2420_DEF_RFPOWER=31"
BUILD_EXTRA_DEPS = RadioCountMsg.py RadioCountMsg.class #DataMsg.java Mts400Tester.class
CLEAN_EXTRA = RadioCountMsg.py RadioCountMsg.class RadioCountMsg.java #$(BUILD_EXTRA_DEPS) Mts400Tester.class

RadioCountMsg.py: RadioCountToLeds.h
	mig python -target=$(PLATFORM) $(CFLAGS) -python-classname=RadioCountMsg RadioCountToLeds.h radio_count_msg -o $@

RadioCountMsg.class: RadioCountMsg.java
	javac RadioCountMsg.java

RadioCountMsg.java: RadioCountToLeds.h
	mig java -target=$(PLATFORM) $(CFLAGS) -java-classname=RadioCountMsg RadioCountToLeds.h radio_count_msg -o $@

#DataMsg.java: DataMsg.h
#	mig -target=null -java-classname=DataMsg java DataMsg.h datamsg -o $@

#Mts400Tester.class: Mts400Tester.java DataMsg.java
#	javac Mts400Tester.java

include $(MAKERULES)

