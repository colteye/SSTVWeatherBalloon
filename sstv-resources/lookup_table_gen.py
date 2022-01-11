rb_lut = []
g_lut = []
clk_speed = 160000000 # in hz
start_freq = 1500

BIT_24_SIZE = 256
RB_MAX_SIZE = 32
G_MAX_SIZE = 64


def freq_2_period(clk_speed, freq):
    period = int(clk_speed / freq)
    
    '''while clk_speed % period != 0:
        period += 1
    
    freq_back = clk_speed / period
    print(freq_back)'''
    
    if (freq % 2) != 0:
        freq += 1
    
    #return int(round(clk_speed / freq)) - 1
    return float(freq)





rb_lutp = []
g_lutp = []

CLK_SAMPLERATE = 80000

def freq_2_period2(clk_speed, freq):
    period = int(clk_speed / freq)
    return period

for color_byte in range(RB_MAX_SIZE):
    rb_lutp.append( str( freq_2_period(CLK_SAMPLERATE, start_freq + (color_byte *  (BIT_24_SIZE / RB_MAX_SIZE) * 3.1372549)) ))
    
for color_byte in range(G_MAX_SIZE):
    g_lutp.append( str( freq_2_period(CLK_SAMPLERATE, start_freq + (color_byte * (BIT_24_SIZE / G_MAX_SIZE) * 3.1372549)) ))



print("\nMartin 1200HZ PERIOD")
print(f"\n#define SSTV_1100HZ_PERIOD {str( freq_2_period(CLK_SAMPLERATE, 1100 ))}")
print(f"\n#define SSTV_1200HZ_PERIOD {str( freq_2_period(CLK_SAMPLERATE, 1200 ))}")
print(f"#define SSTV_1300HZ_PERIOD {str( freq_2_period(CLK_SAMPLERATE, 1300 ))}")
print(f"#define SSTV_1500HZ_PERIOD {str( freq_2_period(CLK_SAMPLERATE, 1500 ))}")
print(f"#define SSTV_1900HZ_PERIOD {str( freq_2_period(CLK_SAMPLERATE, 1900 ))}")

for color_byte in range(RB_MAX_SIZE):
    rb_lut.append( str( freq_2_period(clk_speed, start_freq + int(round(color_byte *  (BIT_24_SIZE / RB_MAX_SIZE) * 3.1372549))) ) )
    
for color_byte in range(G_MAX_SIZE):
    g_lut.append( str( freq_2_period(clk_speed, start_freq + int(round(color_byte * (BIT_24_SIZE / G_MAX_SIZE) * 3.1372549))) ) )
   
print (f"\nstatic const uint32_t RB_FREQ_LUT[{RB_MAX_SIZE}] = {{{','.join(rb_lut)}}};")

print (f"\nstatic const uint32_t G_FREQ_LUT[{G_MAX_SIZE}] = {{{','.join(g_lut)}}};")

print("\nMartin Cycles (ms)")
print(f"\n#define SSTV_MARTIN_2_PULSE_PERIOD { int(round((CLK_SAMPLERATE / 1000) * 4.862))}")
print(f"#define SSTV_MARTIN_2_PORCH_PERIOD { int(round((CLK_SAMPLERATE / 1000) * 0.572))}")
print(f"#define SSTV_MARTIN_2_SEPARATOR_PERIOD { int(round((CLK_SAMPLERATE / 1000) * 0.572))}")
print(f"#define SSTV_MARTIN_2_SCAN_PERIOD { int(round((CLK_SAMPLERATE / 1000) * 0.2288))}")


print(f"\n#define SSTV_300MS_PERIOD { int(round((CLK_SAMPLERATE / 1000) * 300))}")
print(f"#define SSTV_10MS_PERIOD { int(round((CLK_SAMPLERATE / 1000) * 10))}")
print(f"#define SSTV_30MS_PERIOD { int(round((CLK_SAMPLERATE / 1000) * 30))}")


#~3632 hz

print(f"\n#define TIMER0_PERIOD {str( int((clk_speed / 2) / (1000000  / 430)) )}")


'''275.3 * (clk_speed / 2) / 1000 000 = c/usec


275.3 * (clk_speed / (2 * div) ) / 1000 000 = 1

(clk_speed / 2) / (1000 000  / 275.3) = div

0.2753



CLK / X ='''

print (f"\nstatic const float RB_PERIOD_LUT[{RB_MAX_SIZE}] = {{{','.join(rb_lutp)}}};")

print (f"\nstatic const float G_PERIOD_LUT[{G_MAX_SIZE}] = {{{','.join(g_lutp)}}};")




