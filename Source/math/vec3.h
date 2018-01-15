#ifndef __GWU_VEC__
#define __GWU_VEC__

//================================
// vec3
//================================
class vec3 {
public :
	float x, y, z;

public :
	vec3();
	vec3( float x, float y, float z );

	vec3 &			set( float x, float y, float z );

	vec3 &			zero( void );
	
	vec3			operator- ( void ) const;
	vec3			operator+ ( void ) const;

	vec3			operator+ ( const vec3 &v ) const;
	vec3			operator- ( const vec3 &v ) const;
	vec3			operator* ( float scalar ) const;
	vec3			operator/ ( float scalar ) const;
	
	vec3 &			operator= ( const vec3 &v );
	vec3 &			operator+=( const vec3 &v );
	vec3 &			operator-=( const vec3 &v );	
	vec3 &			operator*=( float scalar );
	vec3 &			operator/=( float scalar );

	float &			operator[]( int index );
	const float &	operator[]( int index ) const;

	float			dot( const vec3 &v ) const;

	float			magnitude( void ) const;
	vec3 &			normalize( void );
	vec3 &			cross(const vec3 &v) const;

	float *			ptr( void );
	const float *	ptr( void ) const;

public :
	friend vec3		operator*( float scalar, const vec3 &v );
};

vec3::vec3() {
	x = y = z = 0;
}

vec3::vec3( float x, float y, float z ) {
	this->x = x;
	this->y = y;
	this->z = z;
}

vec3 &vec3::set( float x, float y, float z ) {
	this->x = x;
	this->y = y;
	this->z = z;

	return *this;
}

vec3 &vec3::zero( void ) {
	x = y = z = 0;

	return *this;
}

vec3 vec3::operator-( void ) const {
	return vec3( -x, -y, -z );
}

vec3 vec3::operator+( void ) const {
	return vec3( x, y, z );
}

vec3 vec3::operator+( const vec3 &v ) const {
	return vec3( x + v.x, y + v.y, z + v.z );
}

vec3 vec3::operator-( const vec3 &v ) const {
	return vec3( x - v.x, y - v.y, z - v.z );
}

vec3 vec3::operator*( float scalar ) const {
	return vec3( x * scalar, y * scalar, z * scalar );
}

vec3 vec3::operator/( float scalar ) const {
	float inv = 1.0f / scalar;
	return vec3( x * inv, y * inv, z * inv );
}

vec3 &vec3::operator=( const vec3 &v ) {
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}

vec3 &vec3::operator+=( const vec3 &v ) {
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

vec3 &vec3::operator-=( const vec3 &v ) {
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

vec3 &vec3::operator*=( float scalar ) {
	x *= scalar;
	y *= scalar;
	z *= scalar;
	return *this;
}

vec3 &vec3::operator/=( float scalar ) {
	float inv = 1.0f / scalar;
	x *= inv;
	y *= inv;
	z *= inv;
	return *this;
}

float &vec3::operator[]( int index ) {
	assert( index >= 0 && index < 3 );
	return (&x)[ index ];
}

const float &vec3::operator[]( int index ) const {
	assert( index >= 0 && index < 3 );
	return (&x)[ index ];
}

float vec3::dot( const vec3 &v ) const {
	return x * v.x + y * v.y + z * v.z;
}

float vec3::magnitude( void ) const {
	return sqrtf( x * x + y * y + z * z );
}

vec3 &vec3::normalize( void ) {
	float mag = sqrtf( x * x + y * y + z * z );

	if ( mag < 1e-6f ) {
		x = y = z = 0;
	} else {
		float inv = 1.0f / mag;

		x *= inv;
		y *= inv;
		z *= inv;
	}

	return *this;
}

float *vec3::ptr( void ) {
	return &x;
}

const float *vec3::ptr( void ) const {
	return &x;
}

vec3 &vec3::cross(const vec3 &v) const {
	return vec3((y * (v.z) - z * (v.y)), (z * (v.x) - x * (v.z)), (x * (v.y) - y * (v.x)));
}

vec3 operator*( float scalar, const vec3 &v ) {
	return vec3( v.x * scalar, v.y * scalar, v.z * scalar );
}

#endif // __GWU_VEC__