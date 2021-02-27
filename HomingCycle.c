switch (currState){
        case HOMING_CYCLE:
            // flash the LED until
            digitalWrite(DO_RUNNING,!digitalRead(DO_RUNNING);
            msDelay(500);

            // right now there is no way of exiting an error condition
            // this is something to be decided
            // for examlple could have the emergency stop button be latching and you need to undo it and then press start?
        break;