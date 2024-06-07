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
        # for i in range(var_num):
        #    data[i].append(float(split_l[1 + 2*i]))
        data[0].append(float(split_l[1 + 2*0]))
        data[1].append(float(split_l[1 + 2*1]))
        data[2].append(float(split_l[1 + 2*2]))
        data[3].append(float(split_l[1 + 2*3]))
        data[4].append(float(split_l[1 + 2*4]))
        data[5].append(float(split_l[1 + 2*5]))
        data[6].append(float(split_l[1 + 2*6]))
        data[7].append(float(split_l[1 + 2*7]))

    plt.figure(figsize=(10, 30))
    plt.subplot(411)
    for i in [0, 1]:
        plt.plot(range(len(content)), data[i], label=labels[i])
    plt.title(title)
    plt.xlim([0, len(content)])
    # plt.ylim([-12, 12])
    plt.grid()
    plt.legend()

    plt.subplot(412)
    for i in [2, 3]:
        plt.plot(range(len(content)), data[i], label=labels[i])
    plt.title(title)
    plt.xlim([0, len(content)])
    # plt.ylim([-12, 12])
    plt.grid()
    plt.legend()
    
    
    plt.subplot(413)
    for i in [4, 5]:
        plt.plot(range(len(content)), data[i], label=labels[i])
    plt.grid()
    plt.xlim([0, len(content)])
    # plt.ylim([-12, 12])
    plt.legend()

    plt.subplot(414)
    for i in [6, 7]:
        plt.plot(range(len(content)), data[i], label=labels[i])
    plt.title(title)
    plt.xlim([0, len(content)])
    # plt.ylim([-12, 12])
    plt.grid()
    plt.legend()
    



    # plt.subplot(313)
    # for i in [0, 5]:
    #     plt.plot(range(len(content)), data[i], label=labels[i])
    # plt.grid()
    # plt.legend()

    plt.xlabel(x)
    plt.ylabel(y)
    
    plt.savefig(save_file)

plot_data(
    load_file='week10_data/tuning.txt',
    save_file='week10_data/tuning.png',
    title="tuning",
    x="t",
    y="")