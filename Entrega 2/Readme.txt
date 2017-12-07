CONTROLES:

Flechas del teclado: movimiento 
Barra espaciadora: salto
'q' y 'e': cambio de perspectivas
'o' y 'p': mostrar triangulos del navmesh y grafo / mostrar el suelo normal.

RESUMEN:

Se implementaron los navmeshes, grafos, A* y dem�s. Elaborado con OpenGL y PyGame.
No posee sprites, por eso capaz va un poco m�s lento de lo que deber�a.
Posee 3 estados en su m�quina de estados:
    - Saltando: cuando no consigue al jugador en el grafo (si el jugador salta t�cnicamente
no est� en el grafo
    - Caminando: cuando el jugador est� en el grafo pero est� "lejos", no lo ve.
    - Dirigiendose hacia el personaje: Cuando el jugador est� en el grafo y cerca.

Se tuvieron que adaptar algunos algoritmos para su funcionamiento en python, sin embargo son 
lo m�s gen�ricos posibles. El jugador es el cubo amarillo, los NPCs los cubos azules.