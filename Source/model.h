#ifndef __GWU_MODEL__
#define __GWU_MODEL__
#define N 999

//================================
// ModelFace
//================================
class ModelFace {
public :
	std::vector< int > indices;

public :
	ModelFace() {
	}

	~ModelFace() {
	}
};

//================================
// Model
//================================
class Model {
public :
	std::vector< vec3 > verts;
	std::vector< ModelFace > faces;
	std::vector< vec3 > vertsNormals;

public :
	Model() {
	}

	~Model() {
	}

	//=============================================
	// Load Model
	//=============================================
	void Free( void ) {
		verts.clear();
		faces.clear();
	}

	bool LoadModel( const char *path ) {
		if ( !path ) {
			return false;
		}

		Free();

		// open file
		FILE *fp = fopen( path, "r" );
		if ( !fp ) {
			return false;
		}

		unsigned int numVerts = 0;
		unsigned int numFaces = 0;
		// num of vertices and indices
		fscanf( fp, "data%d%d", &numVerts, &numFaces );

		// alloc vertex and index buffer
		verts.resize( numVerts );
		faces.resize( numFaces );
		vertsNormals.resize(numVerts);

		// read vertices
		for ( unsigned int i = 0; i < numVerts; i++ ) {
			fscanf( fp, "%f%f%f", &verts[ i ].x, &verts[ i ].y, &verts[ i ].z );
		}

		// read indices
		for ( unsigned int i = 0; i < numFaces; i++ ) {
			int numSides = 0;
			fscanf( fp, "%i", &numSides );
			faces[ i ].indices.resize( numSides );

			for ( unsigned int k = 0; k < faces[ i ].indices.size(); k++ ) {
				fscanf( fp, "%i", &faces[ i ].indices[ k ] );
			}
		}

		// close file
		fclose( fp );

		ResizeModel();

		SetNormals();

		return true;
	}

	void SetNormals() {
		for (unsigned int i = 0; i < verts.size(); i++) {
			vec3 EmptyNor;

			for (unsigned int j = 0; j < faces.size(); j++) {
				for (unsigned int k = 0; k < faces[j].indices.size(); k++) {
					if (faces[j].indices[k] == i) {

						vec3 ver3New0 = verts[(faces[j].indices[0])];
						vec3 ver3New1 = verts[(faces[j].indices[1])];
						vec3 ver3New2 = verts[(faces[j].indices[2])];
						vec3 vertCross1 = ver3New1 - ver3New0;
						vec3 vertCross2 = ver3New2 - ver3New1;
						vec3 cross = vertCross1.cross(vertCross2);

						EmptyNor = EmptyNor + cross.normalize();
					}
				}
			}

			vertsNormals[i] = EmptyNor.normalize();

		}
	}

	//=============================================
	// Render Model
	//=============================================
	void DrawEdges2D( void ) {
		glBegin( GL_LINES );
		for ( unsigned int i = 0; i < faces.size(); i++ ) {
			for ( unsigned int k = 0; k < faces[ i ].indices.size(); k++ ) {
				int p0 = faces[ i ].indices[ k ];
				int p1 = faces[ i ].indices[ ( k + 1 ) % faces[ i ].indices.size() ];
				glVertex2fv( verts[ p0 ].ptr() );
				glVertex2fv( verts[ p1 ].ptr() );
			}
		}
		glEnd();
	}

	
	void DrawEdges( void ) {
		glBegin(GL_LINES);
		for ( unsigned int i = 0; i < faces.size(); i++ ) {
			for ( unsigned int k = 0; k < faces[ i ].indices.size(); k++ ) {
				int p0 = faces[ i ].indices[ k ];
				int p1 = faces[ i ].indices[ ( k + 1 ) % faces[ i ].indices.size() ];
				glVertex3fv( verts[ p0 ].ptr() );
				glVertex3fv( verts[ p1 ].ptr() );
			}
		}
		glEnd();
	}

	void DrawPoly(void) {
		
		for (unsigned int i = 0; i < faces.size(); i++) {
			glBegin(GL_POLYGON);
			
			for (unsigned int k = 0; k < faces[i].indices.size(); k++) {
				
				int p0 = faces[i].indices[k];
				glNormal3f(vertsNormals[p0].x, vertsNormals[p0].y, vertsNormals[p0].z);
				glVertex3fv(verts[p0].ptr());

			}
			glEnd();
		}
		
	}

	//=============================================
	// Resize Model
	//=============================================
	// scale the model into the range of [ -0.9, 0.9 ]
	void ResizeModel( void ) {
		// bound
		vec3 min, max;
		if ( !CalcBound( min, max ) ) {
			return;
		}

		// max side
		vec3 size = max - min;

		float r = size.x;
		if ( size.y > r ) {
			r = size.y;
		}
		if ( size.z > r ) {
			r = size.z;
		}

		if ( r < 1e-6f ) {
			r = 0;
		} else {
			r = 1.0f / r;
		}

		// scale
		for ( unsigned int i = 0; i < verts.size(); i++ ) {
			// [0, 1]
			verts[ i ] = ( verts[ i ] - min ) * r;
			
			// [-1, 1]
			verts[ i ] = verts[ i ] * 2.0f - vec3( 1.0f, 1.0f, 1.0f );

			// [-0.9, 0.9]
			verts[ i ] *= 0.9;
		}
	}
	
	bool CalcBound( vec3 &min, vec3 &max ) {
		if ( verts.size() <= 0 ) {
			return false;
		}

		min = verts[ 0 ];
		max = verts[ 0 ];

		for ( unsigned int i = 1; i < verts.size(); i++ ) {
			vec3 v = verts[ i ];

			if ( v.x < min.x ) {
				min.x = v.x;
			} else if ( v.x > max.x ) {
				max.x = v.x;
			}

			if ( v.y < min.y ) {
				min.y = v.y;
			} else if ( v.y > max.y ) {
				max.y = v.y;
			}

			if ( v.z < min.z ) {
				min.z = v.z;
			} else if ( v.z > max.z ) {
				max.z = v.z;
			}
		}

		return true;
	}

	//=============================================
	// Transform Model
	//=============================================
	// scale model
	void Scale( float r ) {
		for ( unsigned int i = 0; i < verts.size(); i++ ) {
			verts[ i ] *= r;
		}
	}

	void Translate( const vec3 &offset ) {
		// translate ...
	}

	void Rotate( float angle ) {
		// rotate ...
	}
};

#endif // __GWU_MODEL__