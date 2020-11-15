void ARDUINO_KERNEL_MAIN() {

  tServoChannel servo_a;
  tServoChannel servo_b;
  hal_io_servo_create_channel(&servo_a, ServoA);
  hal_io_servo_create_channel(&servo_b, ServoB);

  while(true){
    for (int pos = 90; pos <=180; pos += 1) {
      hal_io_servo_write(&servo_a, pos);
      hal_io_servo_write(&servo_b, pos);
      hal_cpu_delay(15);
    }
    for (int pos = 180; pos >= 90; pos -= 1) {
      hal_io_servo_write(&servo_a, pos);
      hal_io_servo_write(&servo_b, pos);
      hal_cpu_delay(15);
    }

  }

  //Exit so we don't
  //loop over and over
  exit(0);
}
