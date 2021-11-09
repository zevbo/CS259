import itertools

f = open('det-test.txt', 'r')
line = f.read()
f.close()

vars = [char + i for (char, i) in itertools.product(['a',
                                                     'b', 'c', 'd', 'e', 'f', 't'], ['1', '2', '3', '4', '5', '6'])]

line = line.replace('[', ',')
line = line.replace(']', '')
line = line.replace('sin', 'Sin')
line = line.replace('cos', 'Cos')
for var in vars:
    line = line.replace('(' + var + ')', '[' + var + ']')

new_line = []
pos_on = (1, 1)
for c in line:
    if c == ",":
        if (pos_on != (1, 1)):
            new_line.append("]")
        new_line.append(
            "\nr" + str(pos_on[0]) + str(pos_on[1]) + " = Simplify[")
        new_col = pos_on[1] % 6 + 1
        new_row = pos_on[0] + (0 if new_col == pos_on[1] + 1 else 1)
        pos_on = (new_row, new_col)
    else:
        new_line.append(c)

clears = "".join(["Clear[" + var + "]\n" for var in vars])


line = clears + ''.join(new_line) + """]\n 
    expr = Det[{
  {r11, r12, r13, r14, r15, r16},
  {r21, r22, r23, r24, r25, r26},
  {r31, r32, r33, r34, r35, r36},
  {r41, r42, r43, r44, r45, r46},
  {r51, r52, r53, r54, r55, r56},
  {r61, r62, r63, r64, r65, r66}}] 
  """

f = open('result.txt', 'w')
f.write(line)
f.close()
