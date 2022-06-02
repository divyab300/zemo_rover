
void readLeftEncoder()
{
  int b = digitalRead(left_motor.get_encB());

  if(b>0)
  {
    left_pos++;
  }
  else
  {
    left_pos--;
  }
}
