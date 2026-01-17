set CLIENTS;
set VERTICES;
set VEHICLES;

param depot;
param Q;
param d{CLIENTS};
param xcoord{VERTICES};
param ycoord{VERTICES};

# Calcular custo diretamente usando função
param c{i in VERTICES, j in VERTICES} := 
    if i = j then 0 else sqrt((xcoord[i]-xcoord[j])^2 + (ycoord[i]-ycoord[j])^2);

var x{VERTICES, VERTICES, VEHICLES} binary;
var u{VERTICES, VEHICLES} >= 0;

minimize Z: sum{k in VEHICLES, i in VERTICES, j in VERTICES: i <> j} c[i,j] * x[i,j,k];

# cliente é atendido exatamente uma vez
s.t. atendimento {i in CLIENTS}:
    sum{k in VEHICLES, j in VERTICES: j <> i} x[i,j,k] = 1;

# conservação de fluxo
s.t. conserv_fluxo {i in CLIENTS, k in VEHICLES}:
    sum{j in VERTICES: j <> i} x[i,j,k] = sum{j in VERTICES: j <> i} x[j,i,k];

# cada veículo sai e retorna ao depósito uma vez
s.t. saida_deposito {k in VEHICLES}:
    sum{j in VERTICES: j <> depot} x[depot,j,k] = 1;

s.t. retorno_deposito {k in VEHICLES}:
    sum{i in VERTICES: i <> depot} x[i,depot,k] = 1;

# restrição de capacidade
s.t. capacidade {k in VEHICLES}:
    sum{i in CLIENTS, j in VERTICES: j <> i} d[i] * x[i,j,k] <= Q;

# restrições MTZ
s.t. mtz_init {k in VEHICLES}: u[depot,k] = 0;

s.t. mtz_bounds {i in CLIENTS, k in VEHICLES}: d[i] <= u[i,k] <= Q;

s.t. mtz {i in CLIENTS, j in CLIENTS, k in VEHICLES: i <> j}:
    u[i,k] - u[j,k] + Q * x[i,j,k] <= Q - d[j];

s.t. no_self_arcs {i in VERTICES, k in VEHICLES}: x[i,i,k] = 0;