# -*- coding: utf-8 -*-
"""
Created on Mon Sep  3 08:25:38 2018

@author: Dylan N. Sugimoto & Gabriel Adriano de Melo
"""

import csv
from anytree import Node
from anytree.exporter import DotExporter
from math import sqrt,sin,cos,atan2,radians
from graphviz import Graph, render
filename = 'australia_2.csv'

#TRanformar csv em dicionario
def csv_to_dict(filename):
    
    #read csv
    reader = csv.DictReader(open(filename,'r'))
    
    csv_dict = {}
    list_dict = []
    #make a list of dictionaries
    #each line of csv is a dictionary with 
    #key = collumn name and value = respectly line value
    for line in reader:
        list_dict.append(line)
        
    #turn into one big dictionary
    for key in list_dict[1].keys():
        csv_dict[key] = tuple(each_dict[key] for each_dict in list_dict)
    
    #add a key posxy wich is a list of tuples (lat,long)
    lat = csv_dict['lat']
    lng = csv_dict['lng']
    pos = []
    for x,y in zip(lat,lng):
         pos.append((x,y))
    csv_dict['posxy'] = pos
    return csv_dict

csv_dict = csv_to_dict(filename)

#Calcular a distancia haversine
def calc_dist_haversine(tupleXY,tupleZW):
    #recebe latitude e longitude em graus 
    
    R = 6371 #raio da terra
    diflat = float(tupleXY[0]) - float(tupleZW[0])
    diflon = float(tupleXY[1]) - float(tupleZW[1])
    diflat = radians(diflat)
    diflon = radians(diflon)
    a = sin(diflat/2)*sin(diflat/2) + cos(radians(float(tupleXY[0]))) * cos(radians(float(tupleZW[0]))) * sin(diflon/2)*sin(diflon/2)
    c = 2 * atan2 ( sqrt(a) , sqrt(1 - a))
    d = R * c
    #retorna a distancia em km
    return d

#Construir o grafo das cidades
def build_cities_graph(csv_dict):
    
    city_graph = Graph('Australia',filename = "australia",format = 'svg')
    cities_name = csv_dict['city']
    for index in range(len(cities_name)):
        
        city_id = index + 1
        if(city_id%2 == 0):
            if(index+2 < len(cities_name)):
                city_graph.edge(cities_name[index],cities_name[index+2])
        else:
            if(index > 0):
                city_graph.edge(cities_name[index],cities_name[index-2])
            if(index + 1 < len(cities_name)):
                city_graph.edge(cities_name[index],cities_name[index+1])
           
    return city_graph

#Guardar informacoes das cidades
class Map:
    
    def __init__(self,csv_dict):
        
        
        self.csv_dict = csv_dict
        self.name2id = self.generate_dict_name2id()
        self.id2pos = self.generate_dict_id2pos()
        
    def generate_dict_name2id(self):
        
        dic = {}
        city_names = self.csv_dict['city']
        for index in range(len(city_names)):
            dic[city_names[index]] = index + 1 
        return dic
    def generate_dict_id2pos(self):

        dic = {}
        city_pos = csv_dict['posxy']
        for index in range(len(city_pos)):
            dic[index+1] = city_pos[index]
        return dic
    
    def change_id2pos(self,id_in):
        return self.id2pos[id_in]
    def change_name2id(self,name):
        return self.name2id[name]
    def change_id2name(self,id_in):
        return self.csv_dict['city'][id_in-1]
    def change_name2pos(self,name):
        return self.change_id2pos(self.change_name2id(name))
    def size(self):
        return len(self.name2id)

#achar as cidades visinhas
def descobrir_filhos_id(pai_id):
    
    filho_id = []
    if(pai_id%2 == 0 ):
        filho_id.append(pai_id -1)
        if(pai_id + 2 <= 219 ):
            filho_id.append(pai_id+2)
        if(pai_id-2 > 0):
            filho_id.append(pai_id-2)
    else:
        if(pai_id -2 > 0):
            filho_id.append(pai_id-2)
        if(pai_id +1 <= 219):
            filho_id.append(pai_id+1)
        if(pai_id+2 <= 219):
            filho_id.append(pai_id+2)
    return filho_id
           
   
def greedy(origem,destino,mapa):

    caminho = []
    objetivo_id = mapa.change_name2id(destino)
    pos_destino = mapa.change_name2pos(destino)
    raiz = Node(origem+ "," +str(calc_dist_haversine(mapa.change_name2pos(origem),pos_destino))[:4],city_id = mapa.change_name2id(origem), peso = calc_dist_haversine(mapa.change_name2pos(origem),pos_destino), escolhido = False)
    achei_solucao = False
    list_custo2node = [(raiz.peso,raiz)]
    list_custo = [raiz.peso]
    iteracao = 1
    list_id = [raiz.city_id]
    while(  (not achei_solucao)):
    
        #Acha o folha de menor peso
        min_custo = min(list_custo)
        list_custo.remove(min_custo)
        for tupla in list_custo2node:
            if tupla[0] == min_custo: 
                list_custo2node.remove(tupla)
                menor_node = tupla[1]
                break      
        #coloca o folha de menor peso na solucao
        caminho.append(menor_node)
        #Descobrir os Filhos
        filho_id = descobrir_filhos_id(menor_node.city_id)
        #constroi lista de ancestrais do menor_node
        #(pai,ancestrais do pai)
        #ancestrais = (menor_node,) + menor_node.ancestors
        #list_id_ancestrais = [ node.city_id for node in ancestrais]
        list_id.append(menor_node.city_id)
        #adicionar filhos na arvore
        for city_id in filho_id:
                
                #Se filho nao eh um ancestral, adiciona na arvore
                if not (city_id in list_id):
                    
                    custo = calc_dist_haversine(mapa.change_id2pos(city_id),pos_destino)
                    node = Node(mapa.change_id2name(city_id) + "," + str(custo)[:4]+",#"+str(iteracao),parent = menor_node,city_id = city_id, peso=custo)
                    list_custo2node.append((custo,node))
                    list_custo.append(custo)
                           
        if(caminho[-1].city_id == objetivo_id):
            achei_solucao = True
            
        iteracao+=1
    return caminho,raiz


def greedy_id(origem,destino,mapa):

    caminho = []
    objetivo_id = mapa.change_name2id(destino)
    pos_destino = mapa.change_name2pos(destino)
    raiz = Node(origem+ "," +str(calc_dist_haversine(mapa.change_name2pos(origem),pos_destino))[:4],city_id = mapa.change_name2id(origem), peso = abs(objetivo_id - mapa.change_name2id(origem)), escolhido = False)
    achei_solucao = False
    list_custo2node = [(raiz.peso,raiz)]
    list_custo = [raiz.peso]
    iteracao = 1
    while(  (not achei_solucao)):
    
        #Acha o folha de menor peso
        min_custo = min(list_custo)
        list_custo.remove(min_custo)
        for tupla in list_custo2node:
            if tupla[0] == min_custo: 
                list_custo2node.remove(tupla)
                menor_node = tupla[1]
                break      
        #coloca o folha de menor peso na solucao
        caminho.append(menor_node)
        if(caminho[-1].city_id == objetivo_id):
            achei_solucao = True
        #Descobrir os Filhos
        filho_id = descobrir_filhos_id(menor_node.city_id)
        #constroi lista de ancestrais do menor_node
        #(pai,ancestrais do pai)
        ancestrais = (menor_node,) + menor_node.ancestors
        list_id_ancestrais = [ node.city_id for node in ancestrais]
        
        #adicionar filhos na arvore
        for city_id in filho_id:
                
                #Se filho nao eh um ancestral, adiciona na arvore
                if not (city_id in list_id_ancestrais):
                    
                    custo = abs(objetivo_id - city_id)
                    node = Node(mapa.change_id2name(city_id) + "," + str(custo)[:4]+",#"+str(iteracao),parent = menor_node,city_id = city_id, peso=custo)
                    list_custo2node.append((custo,node))
                    list_custo.append(custo)

        iteracao+=1
    return caminho,raiz


#IMprimir o caminho e parametros da arvore de busca
def Imprimir_solucao(solucao,filename,mapa):
    
    node_list = []
    parent = solucao
    while (parent):
        node_list.append(parent)
        parent = parent.parent
        
    list_solucao = []
    custo_total = 0
    list_id = []
    for node in reversed(node_list):
        name = [x.strip() for x in node.name.split(',')][0]
        list_id.append(node.city_id)
        if(node.parent):
            list_solucao.append(Node(name,parent = list_solucao[-1]))
            custo_total += calc_dist_haversine(mapa.change_id2pos(node.city_id),mapa.change_id2pos(node.parent.city_id))
        else:
           list_solucao.append(Node(name))

    
    DotExporter(list_solucao[0]).to_dotfile(filename)
    render('dot', 'pdf', filename)
    print("Altura da Arvore de busca: ",node_list[-1].height)
    print("Numero de nós da Arvore de busca",len(node_list[-1].descendants) +1)
    print("Custo Total: ",custo_total)
    print("IDS: ",list_id)

#funcao de custo do A*
def custo_Astar(pointA,pointB,pointC,soma_acumulada):

    
    g = calc_dist_haversine(pointA,pointB) + soma_acumulada
    h =  calc_dist_haversine(pointB,pointC) 
    f = g+h
    return f,g,h


def Astar(origem,destino,mapa):

    caminho = []
    objetivo_id = mapa.change_name2id(destino)
    pos_destino = mapa.change_name2pos(destino)
    raiz = Node(origem+ "," +str(calc_dist_haversine(mapa.change_name2pos(origem),pos_destino)).split(".")[0],city_id = mapa.change_name2id(origem), peso = calc_dist_haversine(mapa.change_name2pos(origem),pos_destino),soma_acumulada = 0)
    list_custo2node = [(raiz.peso,raiz)]
    list_custo = [raiz.peso]
    iteracao = 1
    list_id = [mapa.change_name2id(origem)]
    gcity = {}
    gcity[raiz.city_id] = 0
    while(True):
    
        #Acha o folha de menor peso
        min_custo = min(list_custo)
        list_custo.remove(min_custo)
        for tupla in list_custo2node:
            if tupla[0] == min_custo: 
                list_custo2node.remove(tupla)
                menor_node = tupla[1]
                break      
        #coloca o folha de menor peso na solucao
        caminho.append(menor_node)
        if(caminho[-1].city_id == objetivo_id):
            break
        #Descobrir os Filhos
        filho_id = descobrir_filhos_id(menor_node.city_id)
        #constroi lista de ancestrais do menor_node
        #(pai,ancestrais do pai)
        
        pos_menor_node = mapa.change_id2pos(menor_node.city_id)
        menor_node_somaAcumulada = menor_node.soma_acumulada
        #adicionar filhos na arvore
        for city_id in filho_id:
                pos_node = mapa.change_id2pos(city_id)
                custo,g_visinho,h_visinho = custo_Astar(pos_menor_node,pos_node,pos_destino,menor_node_somaAcumulada)
                
                #Colocar filhos que nao estao na arvore
                if not (city_id in list_id):
                    node_soma_acumulada = menor_node_somaAcumulada + calc_dist_haversine(pos_menor_node,pos_node)
                    node = Node(mapa.change_id2name(city_id) + "," + str(custo).split(".")[0]+",#"+str(iteracao),parent = menor_node,city_id = city_id, peso=custo, soma_acumulada = node_soma_acumulada)
                    list_custo2node.append((custo,node))
                    list_custo.append(custo)
                    gcity[city_id] = g_visinho
                    list_id.append(menor_node.city_id)
                elif(g_visinho < gcity[city_id]):
                    node_soma_acumulada = menor_node_somaAcumulada + calc_dist_haversine(pos_menor_node,pos_node)
                    node = Node(mapa.change_id2name(city_id) + "," + str(custo).split(".")[0]+",#"+str(iteracao),parent = menor_node,city_id = city_id, peso=custo, soma_acumulada = node_soma_acumulada)
                    list_custo2node.append((custo,node))
                    list_custo.append(custo)
                    gcity[city_id] = g_visinho
        iteracao+=1
    return caminho,raiz


"""*-------------------------------START-----------------------------------*"""
Australia = Map(csv_dict)
origem = "Alice Springs"
destino = "Yulara"

"""*----------------------------GREEDY-------------------------------------*"""
print("origem: ",origem," id -> ",Australia.change_name2id(origem))
print("destino: ",destino," id -> ",Australia.change_name2id(destino))
print("Comecando Greedy!")
solucao_greedy, raiz_greedy = greedy(origem,destino,Australia)
print("Imprimindo Arvore greedy")
DotExporter(raiz_greedy).to_dotfile("greedy_arvore")
render('dot', 'pdf', 'greedy_arvore') 
print("Imprimindo Solucao greedy") 
Imprimir_solucao(solucao_greedy[-1],"Solucao_greedy",Australia)

"""*------------------------ASTAR------------------------------------------*"""

print("Comecando A*!")
solucao_A,raiz_A = Astar(origem,destino,Australia)
print("Imprimindo Arvore A")
DotExporter(raiz_A).to_dotfile("A_arvore")
render('dot', 'pdf', 'A_arvore') 
Imprimir_solucao(solucao_A[-1],"Solucao_A",Australia)


"""*------------------------------GREED_ID---------------------------------*"""

print("Comecando Greedy ID!")
solucao_greedy,raiz_greedy=greedy_id(origem,destino,Australia)
print("Imprimindo Arvore greedy ID")
DotExporter(raiz_greedy).to_dotfile("greedy_arvore_ID")
render('dot', 'pdf', 'greedy_arvore_ID') 
print("Imprimindo Solucao greedy ID") 
Imprimir_solucao(solucao_greedy[-1],"Solucao_greedy_ID",Australia)


"""*-----------------------------------------------------------------------*"""
"""
Codigo Djisktra para confirmar o menor caminho entre as cidades
@author: John Washam
From: https://startupnextdoor.com/dijkstras-algorithm-in-python-3/ 
"""

"""*-----------------------------------------------------------------------*"""
import queue  
from collections import namedtuple

Edge = namedtuple('Edge', ['vertex', 'weight'])


class GraphUndirectedWeighted(object):  
    def __init__(self, vertex_count):
        self.vertex_count = vertex_count
        self.adjacency_list = [[] for _ in range(vertex_count)]

    def add_edge(self, source, dest, weight):
        assert source < self.vertex_count
        assert dest < self.vertex_count
        self.adjacency_list[source].append(Edge(dest, weight))
        self.adjacency_list[dest].append(Edge(source, weight))

    def get_edge(self, vertex):
        for e in self.adjacency_list[vertex]:
            yield e

    def get_vertex(self):
        for v in range(self.vertex_count):
            yield v


def dijkstra(graph, source, dest):  
    q = queue.PriorityQueue()
    parents = []
    distances = []
    start_weight = float("inf")

    for i in graph.get_vertex():
        weight = start_weight
        if source == i:
            weight = 0
        distances.append(weight)
        parents.append(None)

    q.put(([0, source]))

    while not q.empty():
        v_tuple = q.get()
        v = v_tuple[1]

        for e in graph.get_edge(v):
            candidate_distance = distances[v] + e.weight
            if distances[e.vertex] > candidate_distance:
                distances[e.vertex] = candidate_distance
                parents[e.vertex] = v
                # primitive but effective negative cycle detection
                if candidate_distance < -1000:
                    raise Exception("Negative cycle detected")
                q.put(([distances[e.vertex], e.vertex]))

    shortest_path = []
    end = dest
    while end is not None:
        shortest_path.append(end)
        end = parents[end]

    shortest_path.reverse()

    return shortest_path, distances[dest]

graph_australia = GraphUndirectedWeighted(220)
for name in csv_dict['id']:
    id_visinhos = descobrir_filhos_id(int(name))
    pos_pai = Australia.change_id2pos(int(name))
    for ids in id_visinhos:
        graph_australia.add_edge(int(name),int(ids),calc_dist_haversine(Australia.change_id2pos(int(ids)),pos_pai))
            

short,dist = dijkstra(graph_australia,5,219)
print("Djisktra Shortest path")
print("Shortest path: ",short)
print("Distance: ",dist)
"""*-----------------------------------------------------------------------*"""