import grafo

def menu_grafos():
  facebook_combined = grafo.Grafo()
  rg300_4730 = grafo.Grafo()
  rome99c = grafo.Grafo()
  toy = grafo.Grafo()
  USA_road_dt_DC = grafo.Grafo()
  USA_road_dt_NY = grafo.Grafo()
  web_Google = grafo.Grafo()

  Algo = 0
  while Algo != 9:
    print('''    \n=== ESCOLHA O GRAFO ===
    [ 1 ] facebook_combined
    [ 2 ] rg300_4730
    [ 3 ] rome99c
    [ 4 ] toy
    [ 5 ] USA_road_dt_DC
    [ 6 ] USA_road_dt_NY 
    [ 7 ] web_Google
    [ 8 ] outro grafo
    [ 9 ] Sair     ''')
    Algo = int(input('\n>>Qual sua opção? '))

    if Algo == 1:
      algoritimo = facebook_combined.ler_arquivo("facebook_combined.txt")
      menu_busca(facebook_combined, algoritimo)
      
    elif Algo == 2:
      algoritimo = rg300_4730.ler_arquivo("rg300_4730.txt")
      menu_busca(rg300_4730, algoritimo)
      
    elif Algo == 3:
      algoritimo = rome99c.ler_arquivo("rome99c.txt")
      menu_busca(rome99c, algoritimo)
      
    elif Algo == 4:
      algoritimo = toy.ler_arquivo("toy.txt")
      menu_busca(toy, algoritimo)
      
    elif Algo == 5:
      algoritimo = USA_road_dt_DC.ler_arquivo("USA-road-dt.DC.txt")
      menu_busca(USA_road_dt_DC, algoritimo)
      
    elif Algo == 6:
      algoritimo = USA_road_dt_NY.ler_arquivo("USA-road-dt.NY.txt")
      menu_busca(USA_road_dt_NY, algoritimo)
      
    elif Algo == 7:
      algoritimo = web_Google.ler_arquivo("web-Google.txt")
      menu_busca(web_Google, algoritimo)

    elif Algo == 8:
      nome_arq = input("Nome do arquivo txt: ")
      novo_grafo = grafo.Grafo()
      algoritimo = novo_grafo.ler_arquivo(nome_arq)
      menu_busca(novo_grafo, algoritimo)
      
      
    elif Algo == 9:
      print('Finalizando...')
      print('===' *16)
      
    else:
      print('Você digitou um número inválido!')
      print('===' *16)
    
def menu_busca(grafo, algoritimo):
  import time
  s = 0
  t = 0
  tempo = 0
  
  s = int(input('Vertice inicial: '))
  t = int(input('Vertice final: '))

  if algoritimo == 1:
    #Busca em Largura
    print()
    inicio = time.time()
    grafo.busca_largura(s, t)
    fim = time.time()
    tempo = (fim - inicio)
    print('Tempo de execusão: %f' %(tempo))
    print('===' *16)
    
  elif algoritimo == 0:
    #Dijkstra
    print()
    inicio = time.time()
    grafo.dijkstra(s, t)
    fim = time.time()
    tempo = (fim - inicio)
    print('Tempo de execusão: %f' %(tempo))
    print('===' *16)
    
  elif algoritimo == 2:    
    #Bellman-Ford
    print()
    inicio = time.time()
    grafo.bellman_ford(s, t)
    fim = time.time()
    tempo = (fim - inicio)
    print('Tempo de execusão: %f' %(tempo))
    print('===' *16)
   
menu_grafos()