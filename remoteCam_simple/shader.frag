

#version 330 core

in vec2 TexCoord;

out vec4 color;

// Texture samplers
uniform sampler2D ourTexture1;


uniform int texImageWidth;
uniform int texImageHeight;

vec2 firstRed = vec2(1,0);
vec3 BayerToRGB( sampler2D source, vec2 pos, int w, int h) {

	pos = floor(pos.xy * vec2(w-1,h-1)) / vec2(w-1,h-1);

    vec3 rgb = vec3(0,0,0);
    vec4 sourceSize = vec4( w, h, 1.0/float(w), 1.0/float(h) );
	vec4 center     = vec4( pos.xy, pos.xy * sourceSize.xy + firstRed);
	
	vec4 xCoord = center.x + vec4( -2.0 * sourceSize.z, -sourceSize.z, sourceSize.z, 2.0 * sourceSize.z);
	vec4 yCoord = center.y + vec4( -2.0 * sourceSize.w, -sourceSize.w, sourceSize.w, 2.0 * sourceSize.w);
	
	#define fetch(x, y) (texture2D(source, vec2(x, y)).r)
	float C = texture2D(source, center.xy).r; // ( 0, 0)

    const vec4 kC = vec4( 4.0,  6.0,  5.0,  5.0) / 8.0;

    // Determine which of four types of pixels we are on.
    vec2 alternate = mod(floor(center.zw) , 2.0);

    vec4 Dvec = vec4(
        fetch(xCoord[1], yCoord[1]),  // (-1,-1)
        fetch(xCoord[1], yCoord[2]),  // (-1, 1)
        fetch(xCoord[2], yCoord[1]),  // ( 1,-1)
        fetch(xCoord[2], yCoord[2])); // ( 1, 1)       
    vec4 PATTERN = (kC.xyz * C).xyzz;

	
    // Can also be a dot product with (1,1,1,1) on hardware where that is
    // specially optimized.
    // Equivalent to: D = Dvec[0] + Dvec[1] + Dvec[2] + Dvec[3];
    Dvec.xy += Dvec.zw;
    Dvec.x  += Dvec.y;
        
    vec4 value = vec4(  
        fetch(center.x, yCoord[0]),   // ( 0,-2)
        fetch(center.x, yCoord[1]),   // ( 0,-1)
        fetch(xCoord[0], center.y),   // (-1, 0)
        fetch(xCoord[1], center.y));  // (-2, 0)
             
    vec4 temp = vec4(
        fetch(center.x, yCoord[3]),   // ( 0, 2)
        fetch(center.x, yCoord[2]),   // ( 0, 1)
        fetch(xCoord[3], center.y),   // ( 2, 0)
        fetch(xCoord[2], center.y));  // ( 1, 0)

/*    return (alternate.y == 0.0) ?
           ((alternate.x == 0.0) ?
                 vec3(C, temp.y, Dvec.w) :
                 vec3(temp.w, C, temp.y)) :
            ((alternate.x == 0.0) ?
                 vec3(temp.y, C, temp.x) :
                 vec3(temp.z, Dvec.w, C));
				 */
    // Even the simplest compilers should be able to constant-fold these to avoid the division.
    // Note that on scalar processors these constants force computation of some identical products twice.
    const vec4 kA = vec4(-1.0, -1.5,  0.5, -1.0) / 8.0;
    const vec4 kB = vec4( 2.0,  0.0,  0.0,  4.0) / 8.0;
    const vec4 kD = vec4( 0.0,  2.0, -1.0, -1.0) / 8.0;        
    
	        
    // Conserve constant registers and take advantage of free swizzle on load
    #define kE (kA.xywz)
    #define kF (kB.xywz)
    
    value += temp;
    
    // There are five filter patterns (identity, cross, checker,
    // theta, phi).  Precompute the terms from all of them and then
    // use swizzles to assign to color channels. 
    //
    // Channel   Matches
    //   x       cross   (e.g., EE G)
    //   y       checker (e.g., EE B)
    //   z       theta   (e.g., EO R)
    //   w       phi     (e.g., EO R)
    
	

    #define A (value[0])
    #define B (value[1])
    #define D (Dvec.x)
    #define E (value[2])
    #define F (value[3])
    
    // Avoid zero elements. On a scalar processor this saves two MADDs and it has no
    // effect on a vector processor.
    PATTERN.yzw += (kD.yz * D).xyy;

    PATTERN += (kA.xyz * A).xyzx + (kE.xyw * E).xyxz;
    PATTERN.xw  += kB.xw * B;
    PATTERN.xz  += kF.xz * F;
    
	

	 rgb = (alternate.y == 0.0) ?
        ((alternate.x == 0.0) ?
            vec3(C, PATTERN.xy) :
            vec3(PATTERN.z, C, PATTERN.w)) :
        ((alternate.x == 0.0) ?
            vec3(PATTERN.w, C, PATTERN.z) :
            vec3(PATTERN.yx, C));
	return rgb;
}
#undef A
#undef B
#undef C
#undef D
#undef E
#undef F


void main()
{
	const mat3 corr_matrix = mat3(	vec3(2.460368f, 0.0f, 0.0f),
									vec3(0.0f, 1.982143f, 0.0f),
									vec3(0.0, 0.0, 2.875449));
	const mat3 corr_matrix_t = transpose(corr_matrix);
	const mat3 corr_matrix_i = inverse(corr_matrix);
	const float multiplier = 1.0f/1.98f;
	vec3 uncorrected = vec3((BayerToRGB( ourTexture1, TexCoord, texImageWidth, texImageHeight))*64.0f*multiplier);
	vec3 corrected = vec3(corr_matrix*uncorrected);
	
	color = vec4(corrected, 1.0f);
	
}

