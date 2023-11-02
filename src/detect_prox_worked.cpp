// Il serait important de soit faire un pseudocode avant de débuter à écrire la partie de code de chacun
// Nous devrions se dispatcher qui s'occupe de quelle(s) partie(s) du code pour ne pas faire en double (ou oublier des parties)

#include <Arduino.h>
#include <LibRobus.h>
#define X 3
#define Y 3

bool detecteur_proximite();
bool detecteur_sifflet();
bool lecture(char direction, int x, int y);

int tableau[X][Y] = {{0b00000101,0b00000010,0b00000111},   
                     {0b00000100,0b00000001,0b00000011},
                     {0b00000001,0b00000110,0b00000100}};


void setup()
{
    BoardInit();
    pinMode(49, INPUT);             // pin 47 est la DEL rouge, soit la gauche du robot
    pinMode(53, INPUT);             // pin 53 est la DEL verte, soit la droite du robot
    pinMode(2, INPUT);              // pin 2 est le detecteur de sifflet
    //attachInterrupt(digitalPinToInterrupt(2, Depart, HIGH));           // commande pour que quand 2 est HIGH, ca part la fonction depart
}

void loop()
{
    // Serial.println(detecteur_sifflet());        // affiche dans le terminal les valeurs lus
    // delay(250);

    Serial.println(lecture('A',2,2));
    delay(500);

    //Serial.println(tableau[1][0]);
    //delay(500);
    
}



bool detecteur_proximite()
{
    bool mur = false;                       // variable s'il y a un mur ou non
    bool val_verte =! digitalRead(53);      // lis l'état des pins
    bool val_rouge =! digitalRead(49);

    if (mur == (val_rouge || val_verte)) {     // des que une des deux DEL recoivent, un mur est detecte
        mur = true;
    }
    return mur;
}



bool detecteur_sifflet()
{
    bool sifflet = digitalRead(2);           // variable de detection du sifflet de 5kHz
    return sifflet;
}



bool lecture (char direction, int x, int y){               // fonction de lecture de la légalité du mouvement
    bool legal = false;
    unsigned char mask;
    int bit_1;
    int bit_2;
    if (direction == 'A'){                    // direction == A, s'il veut avancer
        unsigned char avancer=0b00000001;
        int position = 0;
        mask = 0 << position;                  // mask prend l'arrangement du byte pour l'avant
        bit_1 = (tableau[x][y] & mask) >> position;
        bit_2 = (avancer & mask) >> position;
    }
    else if (direction == 'G'){               // direction == G, s'il veut tourner à gauche
        unsigned char gauche=0b00000010;
        int position = 1;
        mask = 1 << position;                  // mask prend l'arrangement du byte pour l'avant
        bit_1 = (tableau[x][y] & mask) >> position;
        bit_2 = (gauche & mask) >> position;
    }
    else if (direction == 'D'){               // direction == G, s'il veut tourner à droite
        unsigned char droite=0b00000100;
        int position = 2;
        mask = 2 << position;                  // mask prend l'arrangement du byte pour l'avant
        bit_1 = (tableau[x][y] & mask) >> position;
        bit_2 = (droite & mask) >> position;
    }
    else{
        Serial.println("Erreur de direction");    // sécurité si une lettre est entrée par erreur 
    }

    
    if (bit_1 == bit_2){
        legal=true;
    }
    else{
        legal=false;
    }

    return legal;
}