// $Id: RadioCountToLedsC.nc,v 1.7 2010-06-29 22:07:17 scipio Exp $

/*									tab:4
 * Copyright (c) 2000-2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */
 
#include "Timer.h"
#include "RadioCountToLeds.h"
#include "printf.h"
#include "Msp430Adc12.h"
//#include "DataMsg.h"
//#include <stdio.h>
 
/**
 * Implementation of the RadioCountToLeds application. RadioCountToLeds 
 * maintains a 4Hz counter, broadcasting its value in an AM packet 
 * every time it gets updated. A RadioCountToLeds node that hears a counter 
 * displays the bottom three bits on its LEDs. This application is a useful 
 * test to show that basic AM communication and timers work.
 *
 * @author Philip Levis
 * @date   June 6 2005
 */


module RadioCountToLedsC @safe() {
  uses {
    interface Leds;
    interface Boot;
    interface Timer<TMilli> as MilliTimer;

    interface Receive;
    interface AMSend;
    interface SplitControl as AMControl;
    interface Packet;
    interface CC2420Packet;
    interface ActiveMessageAddress as AMAdd;
    interface AMPacket;

    // for ADC
    interface Read<uint16_t> as ReadX;
    interface Read<uint16_t> as ReadY;
    interface Read<uint16_t> as ReadZ;
  
    // for sensing
    //interface Read<uint16_t> as ReadSensor;
  }
}
implementation {

  message_t packet;

  uint8_t int_holder;

  bool locked;
  bool isforme;
  uint16_t counter = 0;
  
  am_addr_t addr;

  event void Boot.booted() {
    call CC2420Packet.setPower(&packet, 31);
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      call MilliTimer.startPeriodic(2000);
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
    // do nothing
  }
  
  event void MilliTimer.fired() {
    counter++;
    dbg("RadioCountToLedsC", "RadioCountToLedsC: timer fired, counter is %hu.\n", counter);
    if (locked) {
      return;
    }
    else {
      //radio_count_msg_t* rcm = (radio_count_msg_t*)call Packet.getPayload(&packet, sizeof(radio_count_msg_t));
      uint16_t* intpayload = (uint16_t*) call Packet.getPayload(&packet, sizeof(uint16_t));
      //if (rcm == NULL) {
      if (intpayload == NULL){
	return;
      }

      //rcm->counter = counter;
      *intpayload = 439;
      //AM_BROADCAST_ADDR for broadcasting
      //if (call AMSend.send(801, &packet, sizeof(radio_count_msg_t)) == SUCCESS) {
      if (call AMSend.send(801, &packet, sizeof(uint16_t)) == SUCCESS){
	dbg("RadioCountToLedsC", "RadioCountToLedsC: packet sent.\n", counter);
        call ReadX.read();
        call ReadY.read();
        call ReadZ.read();
        //call ReadSensor.read();	
	locked = TRUE;
      }
    }
  }

  event message_t* Receive.receive(message_t* bufPtr, 
				   void* payload, uint8_t len) {
    dbg("RadioCountToLedsC", "Received packet of length %hhu.\n", len);

    isforme = call AMPacket.isForMe(bufPtr);
    printf("isforme = %d\n", isforme);
    printfflush();
    if (isforme){
      int_holder = call CC2420Packet.getRssi(&packet);

      printf("Message received at RSSI of %d ", int_holder);
      printf("from %d\n", call AMPacket.source(bufPtr));
      printfflush();
      if (len != sizeof(radio_count_msg_t)) {return bufPtr;}
      else {
        radio_count_msg_t* rcm = (radio_count_msg_t*)payload;
               

        if (rcm->counter & 0x1) {
	  call Leds.led0On();
        }
        else {
	  call Leds.led0Off();
        }
        if (rcm->counter & 0x2) {
	  call Leds.led1On();
        }
        else {
	  call Leds.led1Off();
        }
        if (rcm->counter & 0x4) {
	  call Leds.led2On();
        }
        else {
	  call Leds.led2Off();
        }
        return bufPtr;
      }
    }
    else
      return bufPtr;
  }

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (&packet == bufPtr) {
      locked = FALSE;
      int_holder = call CC2420Packet.getPower(&packet);
      printf("Sending done at %d\n", int_holder);
      addr = call AMAdd.amAddress();
      printf("My address is %u\n", (uint16_t*) addr);
      printfflush();

      //FILE* fp;
      //fp = fopen("/vagrant/test.txt", "w+");
      //fprintf(fp, "pewpewpew\n", fp);
      //fclose(fp);
    }
  }

  event void ReadX.readDone(error_t result, uint16_t data){
    if (result == SUCCESS){
      printf("X = %d\n", data);
      printfflush();
    }
  }

  event void ReadY.readDone(error_t result, uint16_t data){
    if (result == SUCCESS){
      printf("Y = %d\n", data);
      printfflush();
    }
  }

  event void ReadZ.readDone(error_t result, uint16_t data){
    if (result == SUCCESS){
      printf("Z = %d\n", data);
      printfflush();
    }
  }
/*
  event void ReadSensor.readDone(error_t result, uint16_t data){
    if (result == SUCCESS){
      printf("Light: %d\n", data);
      printfflush();
    }
  }
*/
  async event void AMAdd.changed(){
    //do nothing. why is this needed?
  }

}




