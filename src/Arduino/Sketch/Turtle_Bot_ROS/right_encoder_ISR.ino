void readRightEncoder()
{
  int b = digitalRead(right_motor.get_encB());

  if(b>0)
  {
    right_pos++;
  }
  else
  {
    right_pos--;
  }
}
