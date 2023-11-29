# A Monte-Carlo tree search planning for pddl4j


Le dossier ASP est une reproduction du tutoriel disponible ici : http://pddl4j.imag.fr/writing_your_own_planner.html

Le dossier MCP contient le Monte-Carlo tree search planning.
Le programme se trouve dans "src/fr/uga/pddl4j/examples/asp/MCP.java"
Ce fichier implémente un planner de type Monte-Carlo avec l'algorithme Pure Random Walk.

Le fichier MCP.java a été compilé avec la commande suivante dans le dossier MCP: 
javac -d classes -cp lib/pddl4j-4.0.0.jar src/fr/uga/pddl4j/examples/asp/*.java

Le programme a été lancé avec la commande suivante depuis le dossier MCP :
javac -d classes;lib/pddl4j-4.0.0.jar fr.uga.pddl4j.examples.asp.MCP

Le main du programme va utiliser HSP et Monte-Carlo sur les domaines et problèmes PDDL présents dans "src/pddl" (blocks, depot, gripper, logistics), et écrire les temps d'execution et le nombre d'action pour aller de l'état initial à la solution dans "src/pddl/data.csv".
A partir des données du fichier csv, des graphiques ont été réalisés avec le fichier "src/pddl/script.py" afin de comparer les performances des deux algorithmes sur les différents problèmes des différents domaines. Sur les abscisse de ces graphiques, les problèmes sont rangés du plus simple au plus complexe.
Remarque : Les problèmes de type depot on été mis de côtés car trop couteux en temps à résoudre.
Nous avons donc 6 graphiques présents dans "src/pddl": 1 pour le temps et 1 pour le nombre d'actions pour chacuns des domaines blocks, gripper et logistics.
Ils sont compilés dans le fichier graphs.pdf à la racine du projet.
