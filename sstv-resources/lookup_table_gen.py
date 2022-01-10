rb_lut = []
g_lut = []
clk_speed = 160000000 # in hz
start_freq = 1500

BIT_24_SIZE = 256
RB_MAX_SIZE = 31
G_MAX_SIZE = 63


def freq_2_period(clk_speed, freq):
    period = int(clk_speed / freq)
    
    '''while clk_speed % period != 0:
        period += 1
    
    freq_back = clk_speed / period
    print(freq_back)'''
    
    if (freq % 2) != 0:
        freq += 1
    
    #return int(round(clk_speed / freq)) - 1
    return freq


print("\nMartin 1200HZ PERIOD")
print(f"\n#define SSTV_MARTIN_2_1100HZ_PERIOD {str( freq_2_period(clk_speed, 1100 ))}")
print(f"\n#define SSTV_MARTIN_2_1200HZ_PERIOD {str( freq_2_period(clk_speed, 1200 ))}")
print(f"#define SSTV_MARTIN_2_1300HZ_PERIOD {str( freq_2_period(clk_speed, 1300 ))}")
print(f"#define SSTV_MARTIN_2_1500HZ_PERIOD {str( freq_2_period(clk_speed, 1500 ))}")
print(f"#define SSTV_MARTIN_2_1900HZ_PERIOD {str( freq_2_period(clk_speed, 1900 ))}")

for color_byte in range(RB_MAX_SIZE):
    rb_lut.append( str( freq_2_period(clk_speed, start_freq + int(round(color_byte *  (BIT_24_SIZE / RB_MAX_SIZE) * 3.1372549))) ) )
    
for color_byte in range(G_MAX_SIZE):
    g_lut.append( str( freq_2_period(clk_speed, start_freq + int(round(color_byte * (BIT_24_SIZE / G_MAX_SIZE) * 3.1372549))) ) )
   
print (f"\nstatic const uint32_t RB_FREQ_LUT[{RB_MAX_SIZE}] = {{{','.join(rb_lut)}}};")

print (f"\nstatic const uint32_t G_FREQ_LUT[{G_MAX_SIZE}] = {{{','.join(g_lut)}}};")

print("\nMartin Cycles (ms)")
print(f"\n#define SSTV_MARTIN_2_PULSE_PERIOD { int(round((clk_speed / 1000) * 0.862))}")
print(f"#define SSTV_MARTIN_2_PORCH_PERIOD { int(round((clk_speed / 1000) * 0.572))}")
print(f"#define SSTV_MARTIN_2_SEPARATOR_PERIOD { int(round((clk_speed / 1000) * 0.572))}")
print(f"#define SSTV_MARTIN_2_SCAN_PERIOD { int(round((clk_speed / 1000) * 0.2288))}")