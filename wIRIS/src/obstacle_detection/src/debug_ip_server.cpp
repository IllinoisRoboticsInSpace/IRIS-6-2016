#include <stdio.h>
#include <string.h>    //strlen
#include <string>    
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>    //write
#include <pthread.h> //for threading , link with lpthread
#include "data_structure.hpp"


struct data_connection
{
    int sock; 
    unsigned char * data; 
    size_t length; 
    size_t width; 
    size_t height; 
    volatile bool * stop_flag;
    volatile bool * read_image;
    data_connection(int s, unsigned char * d, size_t l, size_t w, size_t h, volatile bool * sf, volatile bool * ri)
    :sock(s),data(d),length(l),width(w),height(h),stop_flag(sf),read_image(ri){}
};
 
void *server_handler(void * pointer);
void *connection_handler(void * pointer); 
 
int debug_ip_server(int port, volatile bool * stop_flag, volatile bool * read_image, unsigned char * data, size_t length, size_t width, size_t height)
{
    pthread_t sniffer_thread;            
    
    if( pthread_create( &sniffer_thread , NULL ,  server_handler , 
            (void*) (new data_connection(port, data, length, width, height, stop_flag, read_image))) < 0)
    {
        puts("debug_ip_server: **** could not create main thread");
        return 1;
    }
    
    return 0;
}


void *server_handler(void * pointer)
{
    data_connection & data_pointer= *(data_connection*)pointer;
    unsigned char * data=data_pointer.data;
    size_t length=data_pointer.length;
    size_t width=data_pointer.width;
    size_t height=data_pointer.height;
    volatile bool * stop_flag=data_pointer.stop_flag;
    volatile bool * read_image = data_pointer.read_image;
    size_t port=data_pointer.sock;
    delete (data_connection*)pointer;
    
    int socket_desc = -1, new_socket = -1 , c;
    struct sockaddr_in server , client;
    char *message;
     
     
    while( !(*stop_flag) ) 
    {
        //close pending
        if(socket_desc>=0) 
        {
            shutdown(socket_desc,SHUT_RDWR);
            close(socket_desc);
        }
        
        //Create socket
        socket_desc = socket(AF_INET , SOCK_STREAM , 0);
        if (socket_desc == -1)
        {
            printf("debug_ip_server: Could not create socket\n");
            continue;
        }
        
        //reusable address
        int true_v = 1;
        if (setsockopt(socket_desc,SOL_SOCKET,SO_REUSEADDR,&true_v,sizeof(int)) < 0 || true_v!=1)
        {
            printf("debug_ip_server: setsockopt error\n");
            continue;
        }
         
        //Prepare the sockaddr_in structure
        server.sin_family = AF_INET;
        server.sin_addr.s_addr = INADDR_ANY;
        server.sin_port = htons( port );
         
        //Bind
        if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
        {
            puts("debug_ip_server: bind failed\n");
            continue;
        }
        puts("debug_ip_server: bind done\n");
         
        //Listen
        if(listen(socket_desc , 3)<0)
        {
            puts("debug_ip_server: listen failed\n");
            continue;
        }    
         
        //Accept and incoming connection
        puts("debug_ip_server: Waiting for incoming connections...\n");
        c = sizeof(struct sockaddr_in);
        while( !(*stop_flag) )
        {
            //close pending
            if(new_socket>=0) 
            {
                shutdown(new_socket,SHUT_RDWR);
                close(new_socket);
            }
            
            //accept
            new_socket = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
            

            if (new_socket<0)
            {
                puts("debug_ip_server: accept failed\n");
                new_socket=-1;
                break;
            }             
        
            pthread_t sniffer_thread;            
            

            if( pthread_create( &sniffer_thread , NULL ,  connection_handler , 
                    (void*) (new data_connection(new_socket, data, length, width, height,0,read_image))) < 0)
            {
                puts("debug_ip_server: could not create conn thread");
                continue;
            }
            
            //forget socket
            new_socket=-1;
            
        }
        
    }
    
    shutdown(socket_desc,SHUT_RDWR);
    close(socket_desc);
         
    return 0;
}

#pragma pack(push,1)
struct BITMAPFILEHEADER {
  uint16_t  bfType;
  uint32_t bfSize;
  uint16_t  bfReserved1;
  uint16_t  bfReserved2;
  uint32_t bfOffBits;
};

BITMAPFILEHEADER header;

#pragma pack(pop)

int write_uint32_t(std::string & s, uint32_t value)
{
    s.append((char*)&value , 4);
}

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "zlib.h"

#define CHUNK 16384

//Compress from sin to sout.
//def() returns Z_OK on success, Z_MEM_ERROR if memory could not be
//allocated for processing, Z_STREAM_ERROR if an invalid compression
//level is supplied, Z_VERSION_ERROR if the version of zlib.h and the
//version of the library linked do not match. 
int deflate_string(const std::string & sin, std::string & sout, int level=Z_DEFAULT_COMPRESSION)
{
    int ret, flush;
    unsigned have;
    z_stream strm;
    unsigned char out[CHUNK];

    /* allocate deflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    ret = deflateInit(&strm, level);
    if (ret != Z_OK)
        return ret;

    /* compress until end of file */
    strm.avail_in = sin.length();
    flush = Z_FINISH;  // to continue: Z_NO_FLUSH;
    strm.next_in = (unsigned char*) sin.c_str();

    /* run deflate() on input until output buffer not full, finish
       compression if all of source has been read in */
    do {
        strm.avail_out = CHUNK;
        strm.next_out = out;
        ret = deflate(&strm, flush);    /* no bad return value */
        assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
        have = CHUNK - strm.avail_out;
        sout+=std::string((char *)out,have);
    } while (strm.avail_out == 0);
    assert(strm.avail_in == 0);     /* all input will be used */
    assert(ret == Z_STREAM_END);        /* stream will be complete */

    /* clean up and return */
    (void)deflateEnd(&strm);
    return Z_OK;
}


/*
 * This will handle connection for each client
 * */
void *connection_handler(void * pointer)
{
    data_connection & data_pointer= *(data_connection*)pointer;
    unsigned char * data_buffer=data_pointer.data;
    size_t length=data_pointer.length;
    size_t width=data_pointer.width;
    size_t height=data_pointer.height;
    volatile bool * read_image = data_pointer.read_image;
    
    //Get the socket descriptor
    int sock = data_pointer.sock;
    //Free the socket pointer
    delete (data_connection*)pointer;
         
    //puts("debug_ip_server: Connection accepted\n");
                     
    const char *message;
    char buffer[100];
    
    uint32_t bitmap_size=length, 
        total_size = 54+bitmap_size;

    char byte[6];
    char code;
    int retval=0;
    int i=0;
    
    //wait for "GET /" command
    while(retval==0 && i<6)
    {
        retval=recv(sock , &byte[i] , 1 , 0);
            if(retval==1)i++,retval--;
    }
    code=byte[5];
    byte[5]=0;
    if(retval!=0 || strcmp("GET /",byte)!=0)
    {
        shutdown(sock,SHUT_RDWR);
        close(sock);
        return 0;
    }
    
    if(code!='?')
    {
        const char * html=
        "<!DOCTYPE html><html><body><h2>IRIS navigation and collision detection</h2>"
        "<img src=\"/?\" style=\"width:540px;height:540px;\" id=\"yourimage\"><script>\n"
        "window.setInterval(function(){\ndocument.getElementById('yourimage').src = \"?\";\n}, 1000);\n"
        "</script></body>"
        "</html>";
        message = "HTTP/1.1 200 OK\r\n"
                "Accept-Ranges: none\r\n"
                "Content-Length: ";
        write(sock , message , strlen(message));
        sprintf(buffer,"%d",(int)strlen(html));
        write(sock , buffer , strlen(buffer));
        message = "\r\n"
                "Keep-Alive: Off\r\n"
                "Connection: Close\r\n"
                "Content-Type: text/html\r\n"
                "Pragma: no-cache, no-store\r\n"
                "Cache-Control: no-cache, no-store\r\n"
                "\r\n";
        write(sock , message , strlen(message));
        write(sock , html , strlen(html));
        sleep(2);
        shutdown(sock,SHUT_RDWR);
        close(sock);
        return 0;
    }


    std::string data="BM";
    
    
    // file size
    write_uint32_t(data , total_size );

    // reserved field (in hex. 00 00 00 00)
    write_uint32_t(data , 0);

    // offset of pixel data inside the image
    write_uint32_t(data , 54);
    
    // -- BITMAP HEADER -- //

    // header size
    write_uint32_t(data , 40 );

    // width of the image
    write_uint32_t(data , width );

    // height of the image
    write_uint32_t(data , height );

    // reserved field
    buffer[0] = 1;
    buffer[1] = 0;

    // number of bits per pixel
    buffer[2] = 24; // 3 byte
    buffer[3] = 0;
    data.append( buffer , 4);
    
    // compression method (no compression here)
    write_uint32_t(data , 0 );

    // size of pixel data
    write_uint32_t(data , bitmap_size );

    // horizontal resolution of the image - pixels per meter (2835)
    buffer[0] = 0;
    buffer[1] = 0;
    buffer[2] = 0b00110000;
    buffer[3] = 0b10110001;

    // vertical resolution of the image - pixels per meter (2835)
    buffer[4] = 0;
    buffer[5] = 0;
    buffer[6] = 0b00110000;
    buffer[7] = 0b10110001;
    data.append(  buffer , 8);

    // color pallette information
    write_uint32_t(data , 0 );

    // number of important colors
    write_uint32_t(data , 0 );
    
    data.append( (char *) data_buffer , length);
    
    *read_image=true;
    
    std::string output;
    
    deflate_string(data,output);
    
    //Reply to the client
    message = "HTTP/1.1 200 OK\r\n"
            "Accept-Ranges: none\r\n"
            "Content-Length: ";
    write(sock , message , strlen(message));
    sprintf(buffer,"%d",output.length());
    write(sock , buffer , strlen(buffer));
    message = "\r\n"
            "Keep-Alive: Off\r\n"
            "Connection: Close\r\n"
            "Content-Type: image/bmp\r\n"
            "Content-Encoding: deflate\r\n"
            "Pragma: no-cache\r\n"
            "Cache-Control: no-cache\r\n"
            "Refresh: 0.5;url=?\r\n"
            "\r\n";    
    write(sock , message , strlen(message));
    write(sock , output.c_str() , output.length());
    sleep(5);
    shutdown(sock,SHUT_RDWR);
    close(sock);
}


