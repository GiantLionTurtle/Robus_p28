
# Guide de style pour le projet de session p28

## Nommenclature

### Fichiers

* Commencent par une majuscule

### Gauche / Droite

* Les valeurs qui représentent la gauche dans un tableau viennent avant celles de la droite
* Les valeurs de la gauche viennent avant celles de la droite dans une signature de fonction

### Classes

* Commencent par une majuscule
* Camelback
* Décrivent spécifiquement une abstraction
* La première accolade est sur la même ligne que le nom de la classe

Exemples:

* MotorController
* PathFollower 

### Fonction

* Commencent par une majuscule
* Camelback
* Se lisent comme
    * Une action
    * Une transformation
    * Une question
* La première accolade est sous le nom de la fonction

Exemples:

* computeNextStep
* readSensor
* is_sensorOn

### Boucles et if/else

* La première accolade est sur la même ligne que l'expression
* Les accolades peuvent être omises pour les if ou l'expression contenue est d'une ligne, si il n'y a pas de else

### Variables

* Commencent par une minuscule
* Variables au nom descriptif
* Pas de variable à une lettre (excepté boucles for)
* Camelback
* Incluent l'unité si il y a ambiguité

Exemples:

* motorSpeed
* remainingTime_ms

### Constantes

* Déclarées const
* Commencent par 'k'

Exemples:

* kMaxVelocity
* kArmMotorPort

## Recommendations techniques

* Privilégier les structures POD (plain old data)
* Garder la logique d'une fonction à son niveau (le bras ne donne pas de commande à la base mobile)
* Privilégier des variables sur le stack