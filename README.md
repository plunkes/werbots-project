# diff-patrol

![Robô Pioneer 3-AT patrulhando uma arena no Webots, com navegação A* e detecção de anomalias](./autonomo.gif)

## O que faz

- Segue uma rota de waypoints usando **A\*** sobre um *occupancy grid* do mapa.
- Suaviza a rota (string-pulling) e desacelera nas curvas para não cortar quinas.
- Fallback **reativo** com LIDAR para não bater; replaneja o A\* ao sair do desvio.
- Detecta anomalias pela câmera: **cubo vermelho** (registra) e **pessoa** (acompanha 10s).
- Replaneja o A\* após registrar cada anomalia.
- Registra a trajetória em `trajetoria_gps.txt`.

## Estrutura

```
worlds/pioneer3at-trab-2026-v1.wbt   mundo (arena, obstáculos, paredes, pedestre)
controllers/
  Astar_controller/                  navegação A* + reativo + visão (controlador do robô)
  random_walk/                       pedestre (random walk preso a um quadrado 10x10)
```

## Como rodar

1. Abrir `worlds/pioneer3at-trab-2026-v1.wbt` no Webots.
2. No robô `PIONEER_3AT`, usar o controlador `Astar_controller`.
3. Play. Tecla **F** na janela 3D encerra e fecha o log.

Após rodar e clicar F, o arquivo será salvo dentro do controlador Astar_controller ->
```
controllers/Astar_contoller/trajetoria_gps.txt
```
