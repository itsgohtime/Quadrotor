import matplotlib.pyplot as plt

def plot_data(load_file, save_file, title, x, y):
    file = open(load_file, "r")
    content = file.readlines()
    
    var_num = len(content[0].split(" ")) // 2
    labels = []
    for i in range(var_num):
        labels.append(content[0].split(' ')[2 * i])
    
    data = [[] for _ in range(var_num)]
    data.append([])
    for l in content:
        split_l = l.replace('\n', '').split(' ')
        for i in range(var_num):
           data[i].append(float(split_l[1 + 2*i]))

    plt.figure(figsize=(10,10))
    plt.subplot(211)
    for i in [1, 2]:
        plt.plot(range(len(content)), data[i], label=labels[i])
    plt.title(title)
    plt.xlim([0, len(content)])
    plt.grid()
    plt.legend()
    plt.grid()
    
    plt.subplot(212)
    for i in [0]:
        plt.plot(range(len(content)), data[i], label=labels[i])
        
    plt.axhline(y=0, color='r', linestyle='-')
    plt.legend()

    plt.xlabel(x)
    plt.ylabel(y)
    
    plt.savefig(save_file)

plot_data(
    load_file='week5_data/milestone5.txt',
    save_file='week5_data/milestone5.png',
    title="milestone5",
    x="t",
    y="")