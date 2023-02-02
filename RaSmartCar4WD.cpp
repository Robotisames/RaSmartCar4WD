#include <RaSmartCar4WD.h>

/**
 * Constructeur de la classe RaSmartCar4WD.
 * 
 * @see https://robotisames.com/robots/41-kit-robot-voiture-4wd-multi-bt-v2-pour-arduino.html
 */
RaSmartCar4WD::RaSmartCar4WD()
{
  debug = false;
  rcHandler = new RaKsRemoteControl(PIN_IR_RECEIVER);
  ledMatrix = new LedMatrixAiP1640(PIN_MATRIX_CLOCK, PIN_MATRIX_DATA);
  distSensor = new SR04(PIN_ECHO, PIN_TRIGGER);
  showSymbols = true;
}

/**
 * @brief l'objet de gestion de la Smart Car.
 * A noter que par défaut :
 *  - le debug = false,
 *  - la vitesse = 0,
 *  - l'unité de mesure (pour le capteur ultrason) est le cm.
 *  - le servomoteur est à 90°C,
 *  - la smart car affiche des symboles sur la matrice de LED.
 */
void RaSmartCar4WD::init()
{
  pinMode(PIN_LED, OUTPUT);

  // Servomotor
  pinMode(PIN_SERVO, OUTPUT);

  // Tracking sensor
  pinMode(PIN_TRACKING_LEFT, INPUT);
  pinMode(PIN_TRACKING_MIDDLE, INPUT);
  pinMode(PIN_TRACKING_RIGHT, INPUT);

  // Ultrasonic sensor
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Motors
  pinMode(PIN_MOTOR_L_CTRL, OUTPUT);
  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_CTRL, OUTPUT);
  pinMode(PIN_MOTOR_R_PWM, OUTPUT);

  // LED Matrix
  pinMode(PIN_MATRIX_CLOCK, OUTPUT);
  pinMode(PIN_MATRIX_DATA, OUTPUT);

  servoHead.attach(PIN_SERVO);
  setSpeed(0);
  distanceUnit = DIST_UNIT_CM;
  Serial.begin(9600);
  rcHandler->init();
  ledMatrix->init();
  setServoAngle(90);
}

/**
 * @brief Active/désactive le mode debug.
 * 
 * @param dbg true = active.
 */
void RaSmartCar4WD::setDebug(bool dbg)
{
  debug = dbg;
  rcHandler->setDebug(debug);
}

/**
 * @brief Définit l'angle (entre 0 et 180°) du servomoteur de la tête de la voiture. 
 * Cette méthode utilise de façon explicite la Modulation de Largeur d'Impulsions (MLI, PWM en Anglais).
 * @see https://fr.wikipedia.org/wiki/Modulation_de_largeur_d%27impulsion
 * 
 * @param iAngle l'angle du servomoteur. Un entier compris entre 0 et 180 (inclus).
 */
void RaSmartCar4WD::setServoAnglePWM(int iAngle)
{
  int pulsewidth = iAngle * 11 + 500; // calculate the value of pulse width
  digitalWrite(PIN_SERVO, HIGH);
  delayMicroseconds(pulsewidth);
  // The duration of high level is pulse width
  digitalWrite(PIN_SERVO, LOW);
  delay(20 - pulsewidth / 1000); // the cycle is 20ms, the low level last for the rest of time
}

/**
 * @brief Définit l'angle (entre 0 et 180°) du servomoteur de la tête de la voiture. 
 * 
 * @param iAngle l'angle du servomoteur. Un entier compris entre 0 et 180 (inclus).
 */
void RaSmartCar4WD::setServoAngle(int iAngle)
{
  servoHead.write(iAngle);
}

/**
 * @brief Définit la vitesse (entre 0 et 255) des moteurs à courant continu de la voiture.
 * 
 * @param iSpeed Une vitesse comprise entre 0 et 255 (inclus).
 */
void RaSmartCar4WD::setSpeed(int iSpeed)
{
  if(iSpeed < 0) {
    iSpeed = 0;
  } else if(iSpeed > SPEED_MAX) {
    iSpeed = SPEED_MAX;
  } else {
    speed = iSpeed;
  }
}

/**
 * @brief Actionne les moteurs pour faire avancer la voiture droit devant.
 * Utilisez la méthode setSpeed pour régler la vitesse du moteur.
 * Si la propriété showSymbols=true, la matrice de LED affiche un symbole.
 * 
 * @see Les méthodes setSpeed, setShowSymbols et displayForward.
 */
void RaSmartCar4WD::goForward()
{
  if(showSymbols)
  {
    displayForward();
  }

  digitalWrite(PIN_MOTOR_L_CTRL, HIGH);
  analogWrite(PIN_MOTOR_L_PWM, speed);
  digitalWrite(PIN_MOTOR_R_CTRL, HIGH);
  analogWrite(PIN_MOTOR_R_PWM, speed);
}

/**
 * @brief Actionne les moteurs pour faire avancer la voiture droit devant.
 * Si la propriété showSymbols=true, la matrice de LED affiche un symbole.
 * 
 * @see Les méthodes setShowSymbols et displayForward.
 * 
 * @param iSpeed Une vitesse comprise entre 0 et 255 (inclus).
 */
void RaSmartCar4WD::goForward(int iSpeed)
{
  if(showSymbols)
  {
    displayForward();
  }

  digitalWrite(PIN_MOTOR_L_CTRL, HIGH);
  analogWrite(PIN_MOTOR_L_PWM, iSpeed);
  digitalWrite(PIN_MOTOR_R_CTRL, HIGH);
  analogWrite(PIN_MOTOR_R_PWM, iSpeed);
}

/**
 * @brief Actionne les moteurs pour faire reculer la voiture.
 * Utilisez la méthode setSpeed pour régler la vitesse du moteur.
 * Si la propriété showSymbols=true, la matrice de LED affiche un symbole.
 * 
 * @see Les méthodes setSpeed, setShowSymbols et displayBackward.
 */
void RaSmartCar4WD::goBackward()
{
  if(showSymbols)
  {
    displayBackward();
  }

  digitalWrite(PIN_MOTOR_L_CTRL, LOW);
  analogWrite(PIN_MOTOR_L_PWM, speed);
  digitalWrite(PIN_MOTOR_R_CTRL, LOW);
  analogWrite(PIN_MOTOR_R_PWM, speed);
}

/**
 * @brief Actionne les moteurs pour faire reculer la voiture.
 * Si la propriété showSymbols=true, la matrice de LED affiche un symbole.
 * 
 * @see Les méthodes setShowSymbols et displayBackward.
 * 
 * @param iSpeed Une vitesse comprise entre 0 et 255 (inclus).
 */
void RaSmartCar4WD::goBackward(int iSpeed)
{
  if(showSymbols)
  {
    displayBackward();
  }

  digitalWrite(PIN_MOTOR_L_CTRL, LOW);
  analogWrite(PIN_MOTOR_L_PWM, iSpeed);
  digitalWrite(PIN_MOTOR_R_CTRL, LOW);
  analogWrite(PIN_MOTOR_R_PWM, iSpeed);
}

/**
 * @brief Actionne les moteurs pour faire tourner la voiture à gauche.
 * Utilisez la méthode setSpeed pour régler la vitesse du moteur.
 * Si la propriété showSymbols=true, la matrice de LED affiche un symbole.
 * 
 * @see Les méthodes setSpeed, setShowSymbols et displayLeft.
 */
void RaSmartCar4WD::turnLeft()
{
  if(showSymbols)
  {
    displayLeft();
  }

  digitalWrite(PIN_MOTOR_L_CTRL, LOW);
  analogWrite(PIN_MOTOR_L_PWM, speed);
  digitalWrite(PIN_MOTOR_R_CTRL, HIGH);
  analogWrite(PIN_MOTOR_R_PWM, speed);
}

/**
 * @brief Actionne les moteurs pour faire tourner la voiture à gauche.
 * Si la propriété showSymbols=true, la matrice de LED affiche un symbole.
 * 
 * @see Les méthodes setShowSymbols et displayLeft.
 * 
 * @param iSpeed Une vitesse comprise entre 0 et 255 (inclus).
 */
void RaSmartCar4WD::turnLeft(int iSpeed)
{
  if(showSymbols)
  {
    displayLeft();
  }

  digitalWrite(PIN_MOTOR_L_CTRL, LOW);
  analogWrite(PIN_MOTOR_L_PWM, iSpeed);
  digitalWrite(PIN_MOTOR_R_CTRL, HIGH);
  analogWrite(PIN_MOTOR_R_PWM, iSpeed);
}

/**
 * @brief Actionne les moteurs pour faire tourner la voiture à droite.
 * Utilisez la méthode setSpeed pour régler la vitesse du moteur.
 * Si la propriété showSymbols=true, la matrice de LED affiche un symbole.
 * 
 * @see Les méthodes setSpeed, setShowSymbols et displayRight.
 */
void RaSmartCar4WD::turnRight()
{
  if(showSymbols)
  {
    displayRight();
  }

  digitalWrite(PIN_MOTOR_L_CTRL, HIGH);
  analogWrite(PIN_MOTOR_L_PWM, speed);
  digitalWrite(PIN_MOTOR_R_CTRL, LOW);
  analogWrite(PIN_MOTOR_R_PWM, speed);
}

/**
 * @brief Actionne les moteurs pour faire tourner la voiture à droite.
 * Si la propriété showSymbols=true, la matrice de LED affiche un symbole.
 * 
 * @see Les méthodes setShowSymbols et displayRight.
 * 
 * @param iSpeed Une vitesse comprise entre 0 et 255 (inclus).
 */
void RaSmartCar4WD::turnRight(int iSpeed)
{
  if(showSymbols)
  {
    displayRight();
  }

  digitalWrite(PIN_MOTOR_L_CTRL, HIGH);
  analogWrite(PIN_MOTOR_L_PWM, iSpeed);
  digitalWrite(PIN_MOTOR_R_CTRL, LOW);
  analogWrite(PIN_MOTOR_R_PWM, iSpeed);
}

/**
 * @brief Arrête les moteurs de la voiture.
 * Si la propriété showSymbols=true, la matrice de LED affiche un symbole.
 * 
 * @see Les méthodes setShowSymbols et displayStop.
 */
void RaSmartCar4WD::stop()
{
  if(showSymbols)
  {
    displayStop();
  }

  digitalWrite(PIN_MOTOR_L_CTRL, LOW);
  analogWrite(PIN_MOTOR_L_PWM, 0);
  digitalWrite(PIN_MOTOR_R_CTRL, LOW);
  analogWrite(PIN_MOTOR_R_PWM, 0);
}

/**
 * @brief Fixe l'angle du servomoteur à 90°, 
 * de sorte à fixer la tête de la voiture correctement et définitivement.
 */
void RaSmartCar4WD::calibrateServo()
{
  setServoAnglePWM(90);
}

/**
 * @brief Allume ou éteint la LED de test (voir la définition du PIN_LED).
 * Le branchement de la LED sert à tester les premiers montages et codes du robot.
 * 
 * @param status true = allume la LED.
 */
void RaSmartCar4WD::switchLed(bool status)
{
  digitalWrite(PIN_LED, status);
}

/**
 * @brief Fait clignoter la LED de test.
 * 
 * @param iDelay temps en millisecondes de la période de clignotement.
 */
void RaSmartCar4WD::blinkLed(int iDelay)
{
  switchLed(true);
  if (debug)
  {
    Serial.println("LED switched ON");
  }
  delay(iDelay);

  switchLed(false);
  if (debug)
  {
    Serial.println("LED switched OFF");
  }
  delay(iDelay);
}

/**
 * @brief Fait clignoter la LED de test en faisant varier son éclairage progressivement.
 */
void RaSmartCar4WD::breathLed()
{
  int i;

  for (i = 0; i < 255; i++)
  {
    analogWrite(PIN_LED, i);
    delay(5);
  }
  for (i = 255; i > 0; i--)
  {
    analogWrite(PIN_LED, i);
    delay(5);
  }
}

/**
 * @brief Récupère la valeur du capteur de gauche de suivi de ligne.
 * 
 * @return int 0 = ligne noire détectée, 1 = pas de détection.
 */
int RaSmartCar4WD::getLeftTrack()
{
  return digitalRead(PIN_TRACKING_LEFT);
}

/**
 * @brief Récupère la valeur du capteur du milieu de suivi de ligne.
 * 
 * @return int 0 = ligne noire détectée, 1 = pas de détection.
 */
int RaSmartCar4WD::getMiddleTrack()
{
  return digitalRead(PIN_TRACKING_MIDDLE);
}

/**
 * @brief Récupère la valeur du capteur de droite de suivi de ligne.
 * 
 * @return int 0 = ligne noire détectée, 1 = pas de détection.
 */
int RaSmartCar4WD::getRightTrack()
{
  return digitalRead(PIN_TRACKING_RIGHT);
}

/**
 * @brief Affiche l'état des 3 capteurs de suivi de ligne dans la console (moniteur).
 */
void RaSmartCar4WD::checkTrack()
{
  int leftTrack = getLeftTrack();
  int midTrack = getMiddleTrack();
  int rightTrack = getRightTrack();

  Serial.print("left:");
  Serial.print(leftTrack);

  Serial.print(" middle:");
  Serial.print(midTrack);

  Serial.print(" right:");
  Serial.println(rightTrack);

  delay(500); // delay in between reads for stability
}

/**
 * @brief Définit l'unité de mesure utilisée pour la distance détectée par le capteur ultrason.
 * Cette distance est donnée par la méthode getDistance.
 * 
 * @see La méthode getDistance.
 * 
 * @param unit l'unité de mesure de la distance. Utilisez les constantes suivantes :
 *  - DIST_UNIT_CM : le centimètre,
 *  - DIST_UNIT_INCH : le pouce.
 */
void RaSmartCar4WD::setDistanceUnit(int unit)
{
  distanceUnit = unit;
}

/**
 * @brief Récupère la distance détectée par le capteur ultrason. 
 * L'unité de la valeur de retour peut être définie par la méthode setDistanceUnit.
 * 
 * @see La méthode setDistanceUnit.
 * 
 * @return float La distance. L'unité par défaut est le centimètre. 
 */
float RaSmartCar4WD::getDistance()
{
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  long duration = pulseIn(PIN_ECHO, HIGH);

  // Convert the time into a distance
  if (distanceUnit == DIST_UNIT_CM)
  {
    return (duration / 2) / 29.1; // Divide by 29.1 or multiply by 0.0343
  }

  if (distanceUnit == DIST_UNIT_INCH)
  {
    return (duration / 2) / 74; // Divide by 74 or multiply by 0.0135
  }

  return 0;
}

/**
 * @brief Permet de vérifier le bon fonctionnement de la télécommande infrarouge.
 * La touche pressée est indiquée dans la caonsole (moniteur).
 */
void RaSmartCar4WD::checkRemoteControl()
{
  if (rcHandler->hasSignal())
  {
    if (rcHandler->isArrowUp())
    {
      Serial.println("Arrow up pressed.");
    }
    else if (rcHandler->isArrowDown())
    {
      Serial.println("Arrow down pressed.");
    }
    else if (rcHandler->isArrowLeft())
    {
      Serial.println("Arrow left pressed.");
    }
    else if (rcHandler->isArrowRight())
    {
      Serial.println("Arrow right pressed.");
    }
    else if (rcHandler->isKeyOk())
    {
      Serial.println("OK pressed.");
    }
    else if (rcHandler->isKey0())
    {
      Serial.println("0 pressed.");
    }
    else if (rcHandler->isKey1())
    {
      Serial.println("1 pressed.");
    }
    else if (rcHandler->isKey2())
    {
      Serial.println("2 pressed.");
    }
    else if (rcHandler->isKey3())
    {
      Serial.println("3 pressed.");
    }
    else if (rcHandler->isKey4())
    {
      Serial.println("4 pressed.");
    }
    else if (rcHandler->isKeyNumber(5))
    {
      Serial.println("5 pressed.");
    }
    else if (rcHandler->isKeyNumber(6))
    {
      Serial.println("6 pressed.");
    }
    else if (rcHandler->isKeyNumber(7))
    {
      Serial.println("7 pressed.");
    }
    else if (rcHandler->isKeyNumber(8))
    {
      Serial.println("8 pressed.");
    }
    else if (rcHandler->isKeyNumber(9))
    {
      Serial.println("9 pressed.");
    }
    else if (rcHandler->isKeyStar())
    {
      Serial.println("Star key pressed.");
    }
    else if (rcHandler->isKeySharp())
    {
      Serial.println("Sharp key pressed.");
    }

    // Serial.println(irCode.value, HEX);
    rcHandler->resume();
  }
  delay(100);
}

/**
 * @brief Permet d'afficher les informations envoyées via l'interface série
 * lors de l'utilisation d'une application connectée en bluetooth.
 * Le résultat est visible sur la console (moniteur).
 */
void RaSmartCar4WD::debugBluetooth()
{
  char btVal;

  if (Serial.available())
  {
    btVal = Serial.read();
    Serial.print("btVal: ");
    Serial.println(btVal);
  }
}

/**
 * @brief Définit le fonctionnement de la voiture pour l'application "keyes 4WD" de Keyestudio.
 * Il est possible de régler la vitesse d'accélération/décélération avec la constante SPEED_STEP.
 * 
 * @see https://play.google.com/store/apps/details?id=com.keyestudio.keyes4wd&hl=en&gl=US
 * 
 * @todo Gérer le mode "anti-drop" pour empêcher la smart car de tomber quand elle arrive au bord d'une table.
 */
void RaSmartCar4WD::enableBluetoothControl()
{
  char btVal;
  int newSpeed;

  if(Serial.available())
  {
    btVal = Serial.read();
  }

  if(debug)
  {
    Serial.print("btVal: ");
    Serial.println(btVal);
  }

  switch (btVal)
  {
  case 'F':
    btMode = BT_MODE_RUN;
    goForward();
    break;
  case 'B':
    btMode = BT_MODE_RUN;
    goBackward();
    break;
  case 'L':
    btMode = BT_MODE_RUN;
    turnLeft();
    break;
  case 'R':
    btMode = BT_MODE_RUN;
    turnRight();
    break;
  case 'a':
    newSpeed = speed + SPEED_STEP;
    if(newSpeed > SPEED_MAX) {
      newSpeed = SPEED_MAX;
    }
    setSpeed(newSpeed);
    break;
  case 'd':
    newSpeed = speed - SPEED_STEP;
    if(newSpeed < 0) {
      newSpeed = 0;
    }
    setSpeed(newSpeed);
    break;
  case 'S':
    // btMode = BT_MODE_RUN;
    Serial.println("Stop");
    stop();
    break;

  case 'G': // anti-drop
    btMode = BT_MODE_ANTI_DROP;
    break;

  case 'X': // line tracking
    btMode = BT_MODE_LINE_TRACKING;
    break;

  case 'Y': // Avoid
    btMode = BT_MODE_AVOID;
    break;

  case 'U': // Following
    btMode = BT_MODE_FOLLOWING;
    break;
  
  default:
    // btMode = BT_MODE_RUN;
    Serial.println("Default -> stop");
    stop();
    break;
  }

  switch (btMode)
  {
  case BT_MODE_ANTI_DROP:
    Serial.println("A faire !!!");
    stop();
    break;
  case BT_MODE_LINE_TRACKING:
    enableLineTracking();
    break;
  case BT_MODE_AVOID:
    enableAvoidObstacles();
    break;
  case BT_MODE_FOLLOWING:
    enableFollowMovingObjects();
    break;
  }
}

/**
 * @brief Permet d'activer ou désactiver les symboles qui s'affichent sur la matrice de LEDs.
 * 
 * @param iShow true = affiche les symboles sur la matrice de LEDs.
 */
void RaSmartCar4WD::setShowSymbols(bool iShow)
{
  showSymbols = iShow;
}

/**
 * @brief Affiche quelque chose sur la matrice de LEDs 16x8.
 * 
 * @see http://dotmatrixtool.com/
 * 
 * @param entries un tableau de 16 x 8 bits.
 */
void RaSmartCar4WD::display(unsigned char entries[])
{
  ledMatrix->display(entries);
}

/**
 * @brief Affiche un emoji qui sourit sur la matrice de LEDs.
 * @see http://dotmatrixtool.com/
 */
void RaSmartCar4WD::displaySmile()
{
  unsigned char smile[] = {0x00,0x00,0x1c,0x02,0x02,0x02,0x5c,0x40,0x40,0x5c,0x02,0x02,0x02,0x1c,0x00,0x00};
  ledMatrix->display(smile);
}

/**
 * @brief Affiche une flèche vers la gauche sur la matrice de LEDs.
 * @see http://dotmatrixtool.com/
 */
void RaSmartCar4WD::displayLeft()
{
  unsigned char left[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0x10,0x44,0x28,0x10,0x44,0x28,0x10,0x00};
  ledMatrix->display(left);
}

/**
 * @brief Affiche une flèche vers la droite sur la matrice de LEDs.
 * @see http://dotmatrixtool.com/
 */
void RaSmartCar4WD::displayRight()
{
  unsigned char right[] = {0x00,0x10,0x28,0x44,0x10,0x28,0x44,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00};
  ledMatrix->display(right);
}

/**
 * @brief Affiche un emoji sur la matrice de LEDs.
 * @see http://dotmatrixtool.com/
 */
void RaSmartCar4WD::displayStart()
{
  unsigned char start[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
  ledMatrix->display(start);
}

/**
 * @brief Affiche une flèche vers le haut sur la matrice de LEDs.
 * @see http://dotmatrixtool.com/
 */
void RaSmartCar4WD::displayForward()
{
  unsigned char front[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x12,0x09,0x12,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
  ledMatrix->display(front);
}

/**
 * @brief Affiche une flèche vers le bas sur la matrice de LEDs.
 * @see http://dotmatrixtool.com/
 */
void RaSmartCar4WD::displayBackward()
{
  unsigned char back[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x48,0x90,0x48,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
  ledMatrix->display(back);
}

/**
 * @brief Affiche le mot "STOP" sur la matrice de LEDs.
 * @see http://dotmatrixtool.com/
 */
void RaSmartCar4WD::displayStop()
{
  unsigned char stop[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
  ledMatrix->display(stop);
}

/**
 * @brief Efface la matrice de LEDs.
 */
void RaSmartCar4WD::clearDisplay()
{
  unsigned char clear[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  ledMatrix->display(clear);
}

/**
 * @brief Active le mode de suivi de ligne au sol de la voiture.
 */
void RaSmartCar4WD::enableLineTracking()
{
  int left = getLeftTrack();
  int middle = getMiddleTrack();
  int right = getRightTrack();

  if(middle == 1)
  {
    goForward(100);
  }
  else
  {
    if(left == 1 && right == 0)
    {
      turnLeft(200);
    }
    else if(left == 0 && right == 1)
    {
      turnRight(200);
    }
    else
    {
      goForward(70);
      delay(9);
      stop();
    }
  }
}

/**
 * @brief Active le mode de suivi d'un objet en mouvement (grâce au capteur ultrason).
 */
void RaSmartCar4WD::enableFollowMovingObjects()
{
  long distance = distSensor->Distance();

  if(debug)
  {
    Serial.println("Distance: " + String(distance));
  }

  if(distance < 8)
  {
    goBackward();
  }
  else if(distance >= 8 && distance < 13)
  {
    stop();
  }
  else if(distance >= 13 && distance < 35)
  {
    goForward();
  }
  else
  {
    stop();
  }
}

/**
 * @brief Active le mode d'évitement d'obstacles. 
 * Lorsqu'un objet est détecté à moins de 20 cm devant le robot, il s'arrête, il "regarde" à gauche, 
 * puis à droite puis tourne du côté où il y a le plus d'espace (d'après le capteur ultrason).
 */
void RaSmartCar4WD::enableAvoidObstacles()
{
  long distance = distSensor->Distance();

  if(debug)
  {
    Serial.println("Distance: " + String(distance));
  }

  if(distance < 20 && distance > 0)
  {
    stop();
    delay(100);
    setServoAngle(180);
    delay(500);

    long distLeft = distSensor->Distance();
    if(debug)
    {
      Serial.println("Distance left: " + String(distLeft));
    }
    delay(100);
    setServoAngle(0);
    delay(500);

    long distRight = distSensor->Distance();
    if(debug)
    {
      Serial.println("Distance right: " + String(distRight));
    }
    delay(100);

    if(distLeft > distRight)
    {
      turnLeft();
    }
    else
    {
      turnRight();
    }
    setServoAngle(90);
    delay(300);
  }
  else
  {
    goForward();
  }
}

/**
 * @brief Active le contrôle par la télécommande infrarouge.
 * Flèche haut = avancer,
 * Flèche bas = reculer,
 * Flèche gauche = tourner à gauche,
 * Flèche droite = you got it ;-),
 * Bouton "OK" = stop.
 * 
 * @todo Gérer la vitesse.
 */
void RaSmartCar4WD::handleRemoteControl()
{
  if (rcHandler->hasSignal())
  {
    if (rcHandler->isArrowUp())
    {
      goForward();
    }
    else if (rcHandler->isArrowDown())
    {
      goBackward();
    }
    else if (rcHandler->isArrowLeft())
    {
      turnLeft();
    }
    else if (rcHandler->isArrowRight())
    {
      turnRight();
    }
    else if (rcHandler->isKeyOk())
    {
      stop();
    }
  
    rcHandler->resume();
  }
  delay(100);
}