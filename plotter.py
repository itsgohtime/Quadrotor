import matplotlib.pyplot as plt

def plot_data(load_file, save_file, title, x, y):
    file = open(load_file, "r")
    content = file.readlines()
    
    var_num = len(content[0].split(" ")) // 2
    labels = []
    for i in range(var_num):
        labels.append(content[0].split(' ')[2 * i])
    
    data = [[] for _ in range(var_num)]
    for l in content:
        split_l = l.replace('\n', '').split(' ')
        for i in range(var_num):
           data[i].append(float(split_l[1 + 2*i]))
    
    for i in range(var_num):
        plt.plot(range(len(content)), data[i], label=labels[i])

    plt.xlabel(x)
    plt.ylabel(y)
    plt.title(title)

    plt.legend()
    plt.savefig(save_file)

plot_data(
    load_file='week3_data/5Hz.txt',
    save_file='week3_data/5Hz.png',
    title="5Hz",
    x="X",
    y="Y")