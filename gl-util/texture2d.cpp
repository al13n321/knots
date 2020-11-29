#include "texture2d.h"
#include <iostream>
using namespace std;

namespace GL {

Texture2D::Texture2D(ivec2 size, GLint internalFormat, GLint filter)
		: size_(size) {
	unsigned int *data;
	data = (unsigned int*) new GLuint[
		((size_.x * size_.y) * 4 * sizeof(unsigned int))];
	memset(data, 0, ((size_.x * size_.y) * 4 * sizeof(unsigned int)));

  glGenTextures(1, &name_);CHECK_GL_ERROR();
	glBindTexture(GL_TEXTURE_2D, name_);CHECK_GL_ERROR();
	glTexImage2D(
		GL_TEXTURE_2D, 0, internalFormat, size_.x, size_.y, 0, GL_RGBA,
		GL_UNSIGNED_BYTE, data);CHECK_GL_ERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
		CHECK_GL_ERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
		CHECK_GL_ERROR();

	delete [] data;
}
Texture2D::Texture2D(
		ivec2 size, GLint internalFormat, GLenum format, GLenum type,
		const GLvoid *data, GLint filter)
		: size_(size) {
	glGenTextures(1, &name_);CHECK_GL_ERROR();
	glBindTexture(GL_TEXTURE_2D, name_);CHECK_GL_ERROR();
	glTexImage2D(
		GL_TEXTURE_2D, 0, internalFormat, size_.x, size_.y, 0,	format, type, data);
		CHECK_GL_ERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
		CHECK_GL_ERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
		CHECK_GL_ERROR();
}
void Texture2D::AssignToUniform(int uniform, int unit) const {
	glActiveTexture(GL_TEXTURE0 + unit);CHECK_GL_ERROR();
	glBindTexture(GL_TEXTURE_2D, name_);CHECK_GL_ERROR();
	glUniform1i(uniform, unit);CHECK_GL_ERROR();
}
void Texture2D::SetFilter(GLint filter) {
	glBindTexture(GL_TEXTURE_2D, name_);CHECK_GL_ERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
		CHECK_GL_ERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
		CHECK_GL_ERROR();
}

void Texture2D::SetPixels(ivec2 pos, ivec2 size, GLenum format, GLenum type, void *data) {
	glBindTexture(GL_TEXTURE_2D, name_);CHECK_GL_ERROR();
	glTexSubImage2D(
		GL_TEXTURE_2D, 0, pos.x, pos.y, size.x, size.y, format, type, data);
		CHECK_GL_ERROR();
}

void Texture2D::SetPixels(GLenum format, GLenum type, void *data) {
	SetPixels(ivec2(0, 0), size_, format, type, data);
}

Texture2D::~Texture2D() {
	glDeleteTextures(1, &name_);CHECK_GL_ERROR();
}

}
