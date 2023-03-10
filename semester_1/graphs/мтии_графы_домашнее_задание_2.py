# -*- coding: utf-8 -*-
"""МТИИ графы домашнее задание 2.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/10OCoYRFsY7BSWKLFL7wc4X8p-aJBnZMH

# Домашнее задание по курсу «Графы и сети» (МТИИ, осень 2021)

Для выполнения домашнего задания создайте дубликат этого файла прямо в Google Colab, заполните в нужных местах своим кодом, расшарьте по ссылке доступ для аккаунта dainiak@gmail.com, а саму ссылку на файл пришлите в соответствующей форме.
"""

import networkx as nx

"""### Задание 1
Построив соответствующую сеть и решив для неё задачу о максимальном потоке любым
 из [имеющихся в networkx алгоритмов]
 (https://networkx.org/documentation/stable/reference/algorithms/flow.html),
  найдите паросочетание максимального размера в заданном неориентированном двудольном
   графе $G$ ниже. Убедитесь, что Ваш код проходит тест ниже.
"""

k = 5 # степень вершин
n = 20 # кол-во вершин
G = nx.Graph()
G.add_nodes_from(range(2*n))
G.add_edges_from(
    (a, b + n) 
    for a,b in nx.random_regular_graph(k, n, 2020).edges()
)
# Построенный выше тестовый граф G — двудольный
# с номерами вершин в одной доле от 0 до n-1
# и в другой доле от n до 2*n-1

# В переменную M сохраните множество рёбер 
# наибольшего паросочетания в графе G
# Ребро — это tuple или set или frozenset,
# содержащее соответствующую пару вершин



M = set()
# Ваш код тут.
#net = nx.max_path(G)


# Это простая проверка того, что M образует паросочетание,
# и что его размер совпадает с эталонным
from functools import reduce
print(1 == len({
    2*len(M),
    len(reduce(lambda e1, e2: set(e1) | set(e2), M)),
    len(nx.bipartite.maximum_matching(G, range(n)))
}))

"""### Задание 2
Построив соответствующую сеть и решив для неё задачу о максимальном потоке, найдите максимально возможное количество не пересекающихся по внутренним **вершинам** путей между вершинами $s$ и $t$ в заданном неориентированном графе $G$. Убедитесь, что Ваш код проходит тест ниже.
"""

def num_disjoint_paths(G, s, t):
    # Ваш код тут.

# Простой тест, он должен проходиться при запуске непосредственно после предыдущей ячейки
G = nx.mycielski_graph(10)
s = 7
t = 700
print(
    num_disjoint_paths(G, s, t) 
    == 
    nx.node_connectivity(G, s, t)
)

"""### Задание 3
Построив поток в соответствующей сети, выберите из данного множества задач `tasks` такое подмножество, суммарная выгода выполнения задач которого максимальна. 

Словарь `prerequisites` содержит для каждой задачи перечень других задач, которые требуется решить перед началом её выполнения (если таковые есть). 

Словарь `profit` содержит информацию о выгоде решения каждой из задач. Выгоды — целочисленные, могут быть как положительные, так и отрицательные, и нулевые.

Также задано множество `compulsory` задач, которые нужно решить в обязательном порядке. Подумайте, и опишите в комментириях, как с минимальными усилиями интегрировать это в рассказанный на лекции алгоритм. Убедитесь, что Ваш код проходит тест ниже.
"""

# Ниже приведён алгоритм решения задачи полным перебором,
# результат работы которого можно считать эталонным
from itertools import combinations

def brute_force_selection(tasks, prerequisites, profit, compulsory):
    best_profit = 0
    best_selection = set()
    for ss in range(1, len(tasks)):
        for c in combinations(tasks, ss):
            if any(t not in c for t in compulsory):
                continue
            for t in c:
                if t in prerequisites:
                    for p in prerequisites[t]:
                        if p not in c:
                            break
                    else:
                        continue
                    break
            else:
                current_profit = sum(profit[t] for t in c)
                if current_profit > best_profit:
                    best_profit = current_profit
                    best_selection = set(c)
    return best_selection

def flow_based_selection(tasks, prerequisites, profit, compulsory):
    # Ваш код тут.

# Простой тест, он должен проходиться при запуске непосредственно после выполнения предыдущей ячейки
import random
random.seed(2020)

tasks = set(range(1, 22))

ranges = [
    (1, len(tasks) // 3), 
    (len(tasks) // 3, 2 * len(tasks) // 3), 
    (2 * len(tasks) // 3, len(tasks))
]

prerequisites = {}
for ri, r in enumerate(ranges):
    if ri == 0:
        continue
    for t in range(*r):
        if random.randrange(1000)/1000 < 0.4:
            prerequisites[t] = set(random.sample(range(*ranges[ri-1]), 4))

profit = {
    task: random.randrange(-7, 15)
    for task in tasks
}

compulsory = set(random.sample(tasks, 2))

print(1 == len(set(
    sum(map(profit.get, 
        method(
            tasks, 
            prerequisites, 
            profit, 
            compulsory
        )
    ))
    for method in [
        brute_force_selection, 
        flow_based_selection
    ]
)))