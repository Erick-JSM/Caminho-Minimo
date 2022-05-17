class Grafo:
  import heapq

  def __init__(self, num_vert = 0, num_arestas = 0, lista_adj = None, mat_adj = None):
    self.num_vert = num_vert
    self.num_arestas = num_arestas
    if lista_adj is None:
      self.lista_adj = [[] for i in range(num_vert)]
    else:
      self.lista_adj = lista_adj
    if mat_adj is None:
      self.mat_adj = [[0 for j in range(num_vert)] for i in range(num_vert)]
    else:
      self.mat_adj = mat_adj
#=========================================================================================
  def add_aresta(self, u, v, w = 1):
    """Adiciona aresta de u a v com peso w"""
    if u < self.num_vert and v < self.num_vert:
      self.num_arestas += 1
      self.mat_adj[u][v] = w
      self.lista_adj[u].append((v, w))
    else:
      print("Aresta invalida!")
#=========================================================================================
  def complementar(self):
    """Cria uma matriz de 0 e percore-la add 1 no lugar dos 0, menos na diagonal principal"""
    m = [[0 for i in range(self.num_vert)] for j in range(self.num_vert)]
    for i in range(len(self.mat_adj)):
      for j in range(len(self.mat_adj)):
        if i != j:
          """Inverte os 1 e 0"""
          if self.mat_adj[i][j] == 0:
            m[i][j] = 1
    return m
    
#=========================================================================================
  def subgrafo(self, g2):
    """Determinar se g2 é um subgrafo de um grafo( Percorrer a matriz de adjacencia g2 se tiver aresta em g2 que não existe em g1 retornar False)"""
    if g2.num_vert >self.num_vert or g2.num_arestas > self.num_arestas:
      return False
    for i in range(len(g2.mat_adj)):
      for j in range(len(self.mat_adj)):
        if g2.mat_adj[i][j] != 0 and self.mat_adj[i][j] == 0:
          return False
    return True
#=========================================================================================
  """Busca em largura em um grafo"""
  def busca_largura(self, s, d):
    
    desc = [0 for i in range(self.num_vert)]  # Marca todos os vertices como não-descobertos
    Q = [s]  #Adiciona o vertice inicial a pilha Q
    R = [s]  #Adiciona o vertice inicial ao vetor R de descobertos
    desc[s] = 1  #Marca o vertice na posição s como descoberto
    custo = 0
    
    while len(Q) != 0:
      u = Q.pop(0)  # u recebe o primeiro elemento de Q, que é retirado da pilha
      for v in self.lista_adj[u]:
        if desc[v[0]] == 0:
          if v[0] == d:
            R.append(v[0])
            custo = custo + 1
            print('Algoritmo: Busca em Largura')
            print('Origem: %d' % (s))
            print('Destino: %d' %(d))
            print('Custo: ', custo)
            print('Caminho: ', R)
            return
          else:
            Q.append(v[0])  # Adiciona v ao final de Q
            R.append(v[0])  # Adiciona v ao final de R
            desc[v[0]] = 1  # Marca o vertice como descoberto
            custo = custo + 1
#=========================================================================================
  """Busca em profundidade em um grafo"""
  def busca_profundidade(self, s, d):
    
    desc = [0 for i in range(self.num_vert)]
    S = [s]
    R = [s]
    desc[s] = 1
    custo = 0
    
    while len(S) != 0:
      u = S[-1]
      flag = False
      for v in self.lista_adj[u]:
        if desc[v[0]] == 0:
          if v[0] == d:
            R.append(v[0])
            custo = custo + 1
            print('Algoritmo: Busca em Profundidade')
            print('Origem: %d' % (s))
            print('Destino: %d' %(d))
            print('Custo: ' , custo)
            print('Caminho: ', R)
            return
          else:
            flag = True
            S.append(v[0])
            R.append(v[0])
            desc[v[0]] = 1
            custo = custo + 1
            break
      if flag == False:
          S.pop()
    return R
    
#=========================================================================================
  """Conexo"""
  def desconexo(self):
    desc = [0 for i in range(self.num_vert)]
    S = [0]
    R = [0]
    desc[0] = 1
    while len(S) != 0:
      u = S[-1]
      flag = False
      for v in self.lista_adj[u]:
        if desc[v[0]] == 0:
          flag = True
          S.append(v[0])
          R.append(v[0])
          desc[v[0]] = 1
          break
      if flag == False:
          S.pop()
    for i in range(len(desc)):
      if desc[i] == 0:
        return True
      else:
        False
#=========================================================================================
  """Ciclo"""
  def ciclo(self):
    desc = [0 for i in range(self.num_vert)]
    S = [0]
    R = [0]
    desc[0] = 1
    while len(S) != 0:
      u = S[-1]
      flag = False
      for v in self.lista_adj[u]:
        if desc[v[0]] == 0:
          flag = True
          S.append(v[0])
          R.append(v[0])
          desc[v[0]] = 1
          break
        for i in range(len(S)):  
          if flag == False:
            S.pop()
            
#=========================================================================================
  """Busca do caminho minimo usando o metodo de Djikstra"""
  def dijkstra(self, s, d):
    
    pred = [None for v in range(self.num_vert)] # Vetor que guarda os predecessores
    dist = [float("inf") for i in range(self.num_vert)]  # Vetor que guarda a distancia
    dist[s] = 0  # Marca o vertice na posição s com distancia 0
    Q = [v for v in range(self.num_vert)]  #Q: lista dos vertices a serem processados

    caminho = []
    count = d
    custo = 0
    
    while Q != []:  # Enquanto a lista Q não for vazia (há vertices a processar)
      # u recebe o vertice de menor distância em Q
      u = None
      min_dist = float("inf")
      for i in Q:
        if dist[i] < min_dist:
          u = i
          min_dist = dist[i]
      Q.remove(u)  # Remove o vertice u da lista Q
      for v,w in self.lista_adj[u]: # Compara o peso da aresta u e v
        if dist[v] > dist[u] + w:
          dist[v] = dist[u] + w
          pred[v] = u

    i = -1
    caminho.append(d)   
    custo = dist[count]
    
    while i != s:
      i = pred[count]
      count = i                  
      caminho.append(i)
    
    caminho.reverse()
    print('Algoritmo: Dijkstra')
    print('Origem: %d' % (s))
    print('Destino: %d' %(d))
    print('Custo: ' , custo)
    print('Caminho: ' , caminho)
#=========================================================================================
  """Busca do caminho minimo usando o metodo de Bellmon Ford"""
  def bellman_ford(self, s, d):

    pred = [None for v in range(self.num_vert)]  # Vetor que guarda os predecessores
    dist = [float("inf") for v in range(self.num_vert)]   # Vetor que guarda a distancia
    dist[s] = 0  # Marca o vertice na posição s com distancia 0

    caminho = []
    count = d
    custo = 0
    
    for i in range(self.num_vert-1):
      flag = False
      for u in range(self.num_vert):
        for v,w in self.lista_adj[u]:  # Compara o peso da aresta u e v
          if dist[v] > dist[u] + w:
            dist[v] = dist[u] + w
            pred[v] = u
            flag = True
      if flag == False:  # Encerra o laço prematuramente
        break
        
    i = -1
    caminho.append(d)    
    custo = dist[count]
    
    while i != s:
      i = pred[count]
      count = i
      caminho.append(i)
    
    caminho.reverse()
    print('Algoritmo: Bellmon Ford')
    print('Origem: %d' % (s))
    print('Destino: %d' %(d))
    print('Custo: ' , custo)
    print('Caminho: ' , caminho)

#=========================================================================================            
  def ler_arquivo(self, nome_arq):
    algoritimo = 1;
    """Le um arquivo"""
    try:
      """Leitura do cabeçalho"""
      arq = open(nome_arq)
      str = arq.readline()
      str = str.split()
      self.num_vert = int(str[0])
      self.num_arestas = int(str[1])
      """Incialização das estruturas de dados"""
      self.lista_adj = [[] for i in range(self.num_vert)]
      self.mat_adj = [[0 for j in range(self.num_vert)] for i in range(self.num_vert)]
      """Le cada aresta"""
      for i in range(self.num_arestas):
        str = arq.readline()
        str = str.split()
        u = int(str[0])
        v = int(str[1])
        w = int(str[2])
        """0 para Dijkstra / 1 para busca em lagura / 2 para Bellmon Ford"""
        
        if algoritimo == 1 and w == 1:
          algoritimo = 1
        else:
          algoritimo = 0
          
        if w < 0:
          algoritimo = 2
          
        self.add_aresta(u,v,w)
      return algoritimo
    except IOError:
      print("Nao foi possivel encontar o arquivo!")
 #========================================================================================= 
