# Robus_p28
Repo pour le robot robus de l'équipe p28


## À faire:

### Base mobile

* ~~Étape 1 de Drivebase::update_concrete: implementer arc_from_targetHeading dans Drivebase.cpp -- Julien~~
    * Un Arc doit contenir un rayon, 
    * Un point de fin
    * Un vecteur tangeant au point de début
    * Une longueur en mètres (devrait pouvoir se calculer avec les 3 autres données?)
    * Voir fig.3

* Tester les maths de calcul d'arcs -- Théo

* ~~Étape 2 de Drivebase::update_concrete, implémenter la fonction velocity_for_point qui calcule le profil d'accélération (voir fig.1)~~

* ~~Étape 4 de Drivebase::update_concrete: Implementer un PID pour corriger l'angle -- Théo~~

* Mettre à jour la base mobile avec les données de zones lorsque non-ambigu
    * Utiliser une intersection de ligne (ligne de séparation de 2 zones) + ligne définie par la position du robot + sa direction

* ~~Déterminer le changement de position du robot selon la vitesse des roues~~

* Trouver comment suivre une ligne

### Logique de jeu

* Ajouter un capteur IR pour la détection du verre dans SensorsState -- Mathieu

* Utiliser ledit capteur pour avoir une meilleur condition pour démarer l'objectif knockCup dans GameState.cpp
    * Pour démarer l'objectif, retourner Objective::Underway

* Déterminer la zone avec les valeurs du Drivebase::state quand ambigu (deux zones consécutives sans marqueur par exemple) (voir fig.2)
    * Créer des "boites" qui définissent les zones et les comparer à la position

* Déterminer si l'objectif un tour en sens horaire est accompli

* Faire la structure de l'objectif shortcut
    * Créer dynamiquement un DrivebasePath

### Terrain

* ~~Mesurer les posision des zones et les mettre dans les constantes de terrain~~
* ~~Mesurer les intersections entre les zones et les mettre dans les constantes de terrain~~

### Misc

* D'autres choses à faire sont marquées &&Figureout&& dans le code pour qu'il soit facile de les trouver avec une fonction de recherche

* ~~Ajouter une fonction print() à SensorsState et à HardwareState~~

* ~~Changer la façon dont le flot de données se produit pour la base mobile ==> gen_hardwareState ne devrait pas mettre à jour le robot! -- Théo~~