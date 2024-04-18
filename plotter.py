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
    data.append([])
    for l in content:
        split_l = l.replace('\n', '').split(' ')
        # for i in range(var_num):
        #    data[i].append(float(split_l[1 + 2*i]))
        data[0].append(25 * float(split_l[1 + 2*0]))
        data[1].append(25 * float(split_l[1 + 2*1]))
        data[2].append(5 * float(split_l[1 + 2*2]))
        data[3].append(float(split_l[1 + 2*3]))
        data[4].append(float(split_l[1 + 2*4]))
    
    for i in [2, 1, 0, 3, 4]:
        plt.plot(range(len(content)), data[i], label=labels[i])

    plt.xlabel(x)
    plt.ylabel(y)

    plt.title(title)

    plt.legend()
    plt.savefig(save_file)

plot_data(
    load_file='week4_data/Milestone4.txt',
    save_file='week4_data/milestone4.png',
    title="Milestone 4",
    x="t",
    y="")