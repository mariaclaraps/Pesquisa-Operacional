# modelo.mod - CVRP com MTZ (usando nomes que evitam colisão)

set CLIENTS;          # clientes (1..n)
set VERTICES;         # vértices (0..n) -> inclui depósito
set VEHICLES;         # veículos (1..m)

param depot;          # índice do depósito (ex: 0)
param Q;              # capacidade de cada veículo
param d{CLIENTS};     # demanda dos clientes
param xcoord{VERTICES};
param ycoord{VERTICES};

# declarar matriz de custos (será preenchida após carregar dados)
param c{VERTICES, VERTICES};

# Variáveis
var x{VERTICES, VERTICES, VEHICLES} binary;   # x[i,j,k] = 1 se veículo k vai de i -> j
var u{VERTICES, VEHICLES} >= 0;               # carga acumulada no veículo k ao chegar em i (MTZ)

# Objetivo: minimizar custo total
minimize Z:
  sum{k in VEHICLES, i in VERTICES, j in VERTICES: i <> j} c[i,j] * x[i,j,k];

# 1) Cada cliente é atendido exatamente uma vez
s.t. atendimento {i in CLIENTS}:
  sum{k in VEHICLES, j in VERTICES: j <> i} x[i,j,k] = 1;

# 2) Conservação de fluxo para clientes (entradas = saídas por veículo)
s.t. conserv_fluxo {i in CLIENTS, k in VEHICLES}:
  sum{j in VERTICES: j <> i} x[i,j,k] = sum{j in VERTICES: j <> i} x[j,i,k];

# 3) Saída e retorno ao depósito para cada veículo
s.t. saida_deposito {k in VEHICLES}:
  sum{j in VERTICES: j <> depot} x[depot,j,k] = 1;

s.t. retorno_deposito {k in VEHICLES}:
  sum{i in VERTICES: i <> depot} x[i,depot,k] = 1;

# 4) Restrição de capacidade por veículo (garante soma das demandas por veículo <= Q)
s.t. capacidade {k in VEHICLES}:
  sum{i in CLIENTS, j in VERTICES: j <> i} d[i] * x[i,j,k] <= Q;

# 5) MTZ para eliminação de sub-rotas (usando carga acumulada u)
s.t. mtz_init {k in VEHICLES}:
  u[depot,k] = 0;

s.t. mtz_bounds {i in CLIENTS, k in VEHICLES}:
  d[i] <= u[i,k] <= Q;

s.t. mtz {i in CLIENTS, j in CLIENTS, k in VEHICLES: i <> j}:
  u[i,k] - u[j,k] + Q * x[i,j,k] <= Q - d[j];

# proibir arcos i->i
s.t. no_self_arcs {i in VERTICES, k in VEHICLES}:
  x[i,i,k] = 0;
