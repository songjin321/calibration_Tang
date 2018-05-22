with open("./Odo.rec") as f:
    content = f.readlines()
for x in content:
    words = x.split()
    words[3] = str(float(words[3])/1000) 
    words[4] = str(float(words[4])/1000)
    outstr = ' '.join(words)
    with open('./Odo_out.rec', 'a') as out_file:
        out_file.write(outstr+"\n")

