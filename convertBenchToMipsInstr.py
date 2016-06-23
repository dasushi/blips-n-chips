import sys, getopt


def main(argv):
   inputfile = ''
   outputfile = ''
   try:
      opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
   except getopt.GetoptError:
      print 'convertBenchToMipsInstr.py -i <inputfile> -o <outputfile>'
      sys.exit(2)
   for opt, arg in opts:
      if opt in ("-i", "--ifile"):
         inputfile = arg
      elif opt in ("-o", "--ofile"):
         outputfile = arg
   f_in = open(inputfile, 'r')
   f_out = open(outputfile, 'w')
   scale = 16 ## equals to hexadecimal
   num_of_bits = 32
   for line in f_in:
      binary = bin(int(line, scale))[2:].zfill(num_of_bits)
      if binary[:6] == "000000":
         f_out.write(special_instr(binary[-6:])+"\n")
      elif binary[:6] == "011100":
         f_out.write(special2_instr(binary[-6:]) + "\n")
      #elif binary[:6] == "010000":
      #   f_out.write(cop0_instr(binary[-6:]) + "\n")
      else:
         instr = get_instr(binary[:6])
         if instr == binary[:6]:
             print(line)
             print(binary[:6] + "  " + binary[-6:])
         f_out.write(get_instr(binary[:6]) + "\n")

def get_instr(instr):
   switcher = {
      "001001":"ADDIU",
      "101011": "SW",
      "000011": "JAL",
      "100011": "LW",
      "000010": "J",
      "000101": "BNE",
      "001010": "SLTI",
      "100100": "LBU",
      "101000": "SB",
      "000100": "BEQ",
      "001111": "LUI",
      "101001": "SH",
   }
   return switcher.get(instr, instr)

def cop0_instr(instr):
   switcher = {
   }
   return switcher.get(instr, instr)

def special2_instr(instr):
   switcher = {
      "000010": "MUL",
   }
   return switcher.get(instr, instr)

def special_instr(instr):
   switcher = {
      "000000": "SLL",
      "000100": "SLLV",
      "100111": "NOR",
      "100101": "OR",
      "000010": "SRL",
      "000110": "SLRV",
      "001101": "BREAK",
      "001100": "SYSCALL",
      "100110": "XOR",
      "100000": "ADD",
      "100001": "ADDU",
      "100100": "AND",
      "011010": "DIV",
      "011011": "DIVU",
      "001001": "JALR",
      "001000": "JR",
      "010000": "MFHI",
      "010010": "MFLO",
      "000001": "MOVCI",
      "001011": "MOVN",
      "001010": "MOVZ",
      "010001": "MTHI",
      "010011": "MTLO",
      "011000": "MULT",
      "011001": "MULTU",
      "101010": "SLT",
      "101011": "SLTU",
      "000011": "SRA",
      "000111": "SRAV",
      "100010": "SUB",
      "100011": "SUBU",
      "001111": "SYNC",
      "110100": "TBQ",
      "110000": "TGE",
      "110001": "TGEU",
      "110010": "TLT",
      "110011": "TLTU",
      "110110": "TNE",
      "001111": "SYNC",

   }
   return switcher.get(instr, instr + " SPECIAL")

if __name__ == "__main__":
   main(sys.argv[1:])
