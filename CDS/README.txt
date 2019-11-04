Pour tester différentes instances de test, renommer un fichier de test en "input.points"
et lancer le build.xml avec cibles compile + run.

Dans la racine du projet on a l'instance de test du TME nommée par défaut "input.points",
2 instances issues du testbed : "input0.points" et "input1.points",
ainsi qu'une instance générée par le Mersenne Twister "inputMersenneTwister.points".
Le reste des instance du testbed est dans le folder "testbeds".

Par défaut l'algorithme utilisé dans le code est l'algo Al Li sans heuristique.
Pour utiliser avec heuristique il faut commenter la ligne 1268 
et décommenter la ligne 1271...