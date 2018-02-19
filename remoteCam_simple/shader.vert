

#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;
layout (location = 2) in vec2 texCoord;


out vec2 TexCoord;
uniform float scale;

void main()
{
	gl_Position = vec4(position, 1.0f);
	//ourColor = vec3(1.0,1.0,1.0);
	// We swap the y-axis by substracing our coordinates from 1. This is done because most images have the top y-axis inversed with OpenGL's top y-axis.
	// TexCoord = texCoord;
	TexCoord = vec2(((texCoord.x-0.5f)*scale) + 0.5f, ((0.5f - texCoord.y)*scale)+0.5f );
}

