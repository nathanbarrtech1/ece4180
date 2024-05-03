const int buttonPin[3][3] = {{A0, A1, A2}, {A3, A4, A5}, {2,  3,  4 }};
const int ledPin[3][3]    = {{5,  6,  7} , {8,  9,  10}, {11, 12, 13}};
bool ledOn[3][3] = {{LOW, LOW, LOW}, {LOW, LOW, LOW}, {LOW, LOW, LOW}};

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  Serial.begin(9600);
  // initialize the LED pin as an output:
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      pinMode(buttonPin[i][j], INPUT);
      pinMode(ledPin[i][j], OUTPUT);
      
    }
  }
  
  while (true)
  {
    if (digitalRead(A4) != HIGH)
    {
      delay(10);
      continue;
    }
    randomSeed(micros());
    long rand1[2] = {random(3), random(3)};
    long rand2[2] = {random(3), random(3)};

    bool rand1CC = (rand1[0] + rand1[1]) % 2 == 0 && rand1[0] != 1 && rand1[1] != 1;
    bool rand2CC = (rand2[0] + rand2[1]) % 2 == 0 && rand2[0] != 1 && rand2[1] != 1;
    
    delay(1000);
    
    digitalWrite(ledPin[rand1[0]][rand1[1]], HIGH);
    digitalWrite(ledPin[rand2[0]][rand2[1]], HIGH);
  
    ledOn[rand1[0]][rand1[1]] = HIGH;
    ledOn[rand2[0]][rand2[1]] = HIGH;
    
    break;
  }
}

void loop() 
{
  

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      buttonState = digitalRead(buttonPin[i][j]);
      
      if (buttonState == HIGH)
      {
        for (int k = -1; k <= 1; k++)
          for (int l = -1; l <= 1; l++)
            if (i + k >= 0 && i + k < 3 && j + l >= 0 && j + l < 3 && (k == 0 || l == 0))
            {
              ledOn[i + k][j + l] = !ledOn[i + k][j + l];
              Serial.println(ledOn[i + k][j + l]);
              digitalWrite(ledPin[i + k][j + l],  ledOn[i + k][j + l]);
            }
        delay(500);
      }
    }
  }   
}
