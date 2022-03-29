##### El nombre del archivo de es lo unico que se debe cambiar para otros ejemplos
with open("ejemplogts.gts") as f:
   lines = f.readlines()
flag = 0
for i,l in enumerate(lines):
   ls = l.split()
   if flag == 0 and len(ls)==2:
      flag = 1
   if flag == 1 and len(ls)==3:
      flag = 2
   if flag != 2:
      continue
   lines[i] = "{} {} {}\n".format(ls[0],ls[2],ls[1])
with open("ejemplogts2.gts","w") as f:
   f.writelines(lines)
######