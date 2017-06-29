# Drone

I) Objectif 

L’Objectif est de fournir aux ESCs le signal de commande qu’ils attendent. Source : https://drive.google.com/open?id=0B5Z2mGW3obq-MUNwbHJhaFBNaTQ 

II) Matériel 
– 1 Raspberry Pi 2 Model B+ 
– 1 Carte micro-SD 
– 1 Gyroscope 3 axes 
– 1 Adaptateur sans fil Windows pour manette Xbox360 sans fil 
– 1 Manette Xbox 360 sans fil 
– 1 Convertisseur Analogique/Numérique ADC Pi Plus 
– 1 SparkFun Logic Level Converter 
– Bi-Directional BOB-12009 

III) Programmation 

Suivre la procédure de programmation : https://drive.google.com/open?id=0B5Z2mGW3obq-WkRORmlqRmhGYlU 
Copier le code source dans /home/pi : https://drive.google.com/open?id=0B5Z2mGW3obq-Rkt3UFFGNFVpRWM 

IV) Câblage des différents éléments 

Se référer à la liste de câblage suivante : https://drive.google.com/open?id=0B5Z2mGW3obq-WkE0N2RITG8zemc 

V) Initialisations des PIDs 

Tutoriel : http://www.ferdinandpiette.com/blog/2012/04/asservissement-en-vitesse-dun-moteur-avec-arduino/ 
Ouvrir /home/pi/drone.py puis modifier à tâtons les valeurs des coefficients. (Dans un premier temps partager une même valeur par coefficient pour les quatre moteurs)
