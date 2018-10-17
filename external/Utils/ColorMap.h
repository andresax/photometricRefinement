#ifndef COLORMAP_H
#define COLORMAP_H

class ColorMap
{
	public:
		enum ColorConstant  {
			Black  =0xff000000,
			Gray   =0xff808080,
			White  =0xffffffff,
			Red    =0xff0000ff,
			Green  =0xff00ff00,
			Blue   =0xffff0000,
			Cyan   		=0xffffff00,
			Yellow 		=0xff00ffff,
			Magenta		=0xffff00ff,
			LightGray	=0xffc0c0c0,
			LightRed	=0xff8080ff,
			LightGreen  	=0xff80ff80,
			LightBlue	=0xffff8080,
			DarkGray	=0xff404040,
			DarkRed		=0xff000040,
			DarkGreen   	=0xff004000,
			DarkBlue	=0xff400000
			};

		void static initColor(float *c, long colorName) {
			long mask[3];
			mask[0]=0x00ff0000; mask[1]=0x0000ff00; mask[2]=0x000000ff;
//			cout << "color:" << colorName << " ";
			for(int i=0;i<3;i++) {
//				c[i]= (colorName&mask[i])/255;
				c[i]= ((colorName&(0x000000ff<<i*8))>>i*8)/255;
//				cout << c[i] << " ";
			}
//			cout << endl;
		}

		inline void static interpolate(const float* c1, const float* c2, const float ratio, float* dest) {
			assert(ratio >=0);
			assert(ratio <=1);
			for(int i=0;i<3;i++)
				dest[i] = c1[i]*ratio+c2[i]*(1.0f-ratio);
		}

		void static jet3(float x, float *rgb) 
		{
			float c1[3];
			float c2[3];
			if(x < 0) { initColor(rgb,Red); return;} 
			if(x < 0.25) { initColor(c1,Red); initColor(c2,Yellow); interpolate(c1,c2,x*4,rgb); return;}
			if(x < 0.50) { initColor(c1,Yellow); initColor(c2,Green); interpolate(c1,c2,(x-0.25)*4,rgb); return;}
			if(x < 0.75) { initColor(c1,Green); initColor(c2,Cyan); interpolate(c1,c2,(x-0.50)*4,rgb); return;}
			if(x < 1) { initColor(c1,Cyan); initColor(c2,Blue); interpolate(c1,c2,(x-0.75)*4,rgb); return;}
			initColor(rgb,Blue); return;

		}

		void static jet2(float x, float *rgb) 
		{
			if(x < 0) { rgb[0] = rgb[1] = rgb[2] = 0; } 
			else if(x < 0.2) { rgb[0] = x/0.2; rgb[1] = 0; rgb[2] = 0; }
			else if(x < 0.4) { rgb[0] = (x-0.2)/0.2; rgb[1] = (0.4-x)/0.2; rgb[2] = 0; }
			else if(x < 0.6) { rgb[0] = 0; rgb[1] = (x-0.4)/0.2; rgb[2] = 0; }
			else if(x < 0.8) { rgb[0] = 0; rgb[1] = (x-0.6)/0.2; rgb[2] = (0.6-x)/0.2; }
			else if(x < 1)   { rgb[0] = 0; rgb[1] = 0; rgb[2] = (1-x)/0.2; }
			else { rgb[0] = rgb[1] = rgb[2] = 1; }
		}

		void static hsv(float x, float *rgb)
		{
			float hue; 
			x = (x < 0) ? 0 : x; 
			x = (x > 1) ? 1 : x; 
			hue = x * 360; 
			HSVtoRGB(hue, 1, 1, rgb);
		}

		void static hot(float x, float *rgb)
		{
			hsv(0.2*x, rgb);
		}

		void static cool(float x, float *rgb)
		{
			hsv(0.2*x+0.6, rgb);
		}

		void static summer(float x, float *rgb)
		{
			hsv(-0.26*x+0.4, rgb);
		}

		void static winter(float x, float *rgb)
		{
			hsv(0.2*x+0.4, rgb);
		}

		void static spring(float x, float *rgb)
		{
			float y = 0.55*x+0.83; 
			y = (y > 1) ? y - 1 : y; 
			hsv(y, rgb);	
		}
		
		void static copper(float x, float *rgb)
		{
			float hue = 38;
			HSVtoRGB(hue, 1, x, rgb);
		}

		void static jet(float x, float *rgb)
		{
			hsv(-0.7*x+0.7, rgb);
		}

		void static RGBtoHSV( float r, float g, float b, float *hsv)
		{
			float min, max, delta;
			float h, s, v; 

			min = std::min(r,std::min(g,b));
			max = std::max(r,std::max(g,b));

			v = max;				// v

			delta = max - min;

			if( max != 0 )
				s = delta / max;		// s
			else {
				// r = g = b = 0		// s = 0, v is undefined
				s = 0;
				h = -1;
				return;
			}
			if (delta==0) delta = 0.001f;
			
			if( r == max )
				h = ( g - b ) / delta;		// between yellow & magenta
			else if( g == max )
				h = 2 + ( b - r ) / delta;	// between cyan & yellow
			else
				h = 4 + ( r - g ) / delta;	// between magenta & cyan
		
			h *= 60;				// degrees
			if( h < 0 )
				h += 360;

			hsv[0] = h; hsv[1] = s; hsv[2] = v; 		
		}

		void static HSVtoRGB(float h, float s, float v, float *rgb )
		{
			int i;
			float f, p, q, t, r, g, b;
		
			if( s == 0 ) {
				// achromatic (grey)
				r = g = b = v;
				return;
			}
		
			h /= 60;			// sector 0 to 5
			i = floor( h );
			f = h - i;			// factorial part of h
			p = v * ( 1 - s );
			q = v * ( 1 - s * f );
			t = v * ( 1 - s * ( 1 - f ) );
		
			switch( i ) {
				case 0:
					r = v;
					g = t;
					b = p;
					break;
				case 1:
					r = q;
					g = v;
					b = p;
					break;
				case 2:
					r = p;
					g = v;
					b = t;
					break;
				case 3:
					r = p;
					g = q;
					b = v;
					break;
				case 4:
					r = t;
					g = p;
					b = v;
					break;
				default:		// case 5:
					r = v;
					g = p;
					b = q;
					break;
			}

			rgb[0] = r; rgb[1] = g; rgb[2] = b; 
		}

};

#endif

