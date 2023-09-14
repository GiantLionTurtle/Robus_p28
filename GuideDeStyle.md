
# Guide de style pour le projet de session p28

## Nommenclature

### Classes

* Commence par une majuscule
* Camelback
* Décrit spécifiquement une abstraction

Exemples:

* MotorController
* PathFollower

### Fonction

* Commence par une majuscule
* Camelback
* Se lit comme
    * Une action
    * Une transformation
    * Une question

Exemples:

* computeNextStep
* readSensor
* is_sensorOn

### Variable

* Commence par une minuscule
* Variables au nom descriptif
* Pas de variable à une lettre (excepté boucles for)
* Camelback
* Inclut l'unité si il y a ambiguité

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