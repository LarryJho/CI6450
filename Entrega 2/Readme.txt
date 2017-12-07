CONTROLES:

Flechas del teclado: movimiento 
Barra espaciadora: salto
'q' y 'e': cambio de perspectivas
'o' y 'p': mostrar triangulos del navmesh y grafo / mostrar el suelo normal.

RESUMEN:

Se implementaron los navmeshes, grafos, A* y demás. Elaborado con OpenGL y PyGame.
No posee sprites, por eso capaz va un poco más lento de lo que debería.
Posee 3 estados en su máquina de estados:
    - Saltando: cuando no consigue al jugador en el grafo (si el jugador salta técnicamente
no está en el grafo
    - Caminando: cuando el jugador está en el grafo pero está "lejos", no lo ve.
    - Dirigiendose hacia el personaje: Cuando el jugador está en el grafo y cerca.

Se tuvieron que adaptar algunos algoritmos para su funcionamiento en python, sin embargo son 
lo más genéricos posibles. El jugador es el cubo amarillo, los NPCs los cubos azules.