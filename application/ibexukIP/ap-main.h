#ifndef APMAIN_H
#define APMAIN_H

//******************************************************************
//******************************************************************
//********** PROCESS HTTP SERVER AUTHORISE CLIENT REQUEST **********
//******************************************************************
//******************************************************************
//This optional function is called every time a HEAD, GET or POST request is received from an HTTP client.  It is useful for
//applications where you want to only provide HTTP to certain clients based on their MAC or IP address, or where you want the
//to option to effectively disconnect clients after some form of initial sign in page.
//
//requested_filename
//	Pointer to a null terminated string containing the filename that will be returned to the client after the driver has finished
//	reading all of the request. Your application may alter this if desired (max length = HTTP_MAX_FILENAME_LENGTH).  Can be ignored if you wish.
//requested_filename_extension
//	Pointer to 3 byte string containing the filename extension.  Your application may alter this if desired or it can be ignored.
//tcp_socket_number
//	Allows your application to identify a user by their mac or IP address (e.g. tcp_socket[tcp_socket_number].remote_device_info.ip_address).
//	Can be ignored if you wish.
//Return
//	1 to authorise the request (http server will process and then respond)
//	0 to reject the request (http server will send a 400 Bad Request response)

BYTE process_http_authorise_request (BYTE *requested_filename, BYTE *requested_filename_extension, BYTE tcp_socket_number);

//***********************************************************************
//***********************************************************************
//********** PROCESS HTTP SERVER OUTPUT DYNAMIC DATA VARIABLES **********
//***********************************************************************
//***********************************************************************
//This optional function is called each time a special tilde ~my_varaible_name- dynamic data marker is found as an HTML page is being
//send to a client following a request.
//
//variable_name
//	Pointer to a null terminated string containing the varaible name (in the source HTML) between the tilde '~' and '-' characters.
//tcp_socket_number
//	Included in case it is helpful for your application to identify a user (e.g. by their mac or IP address,  e.g. tcp_socket[tcp_socket_number].remote_device_info.ip_address).
//	Can be ignored if you wish.
//Return
//	A pointer to the start of a null termianted string which contains the string to be transmitted (max length 100 characters).

BYTE *process_http_dynamic_data (BYTE *variable_name, BYTE tcp_socket_number);


//***********************************************************
//***********************************************************
//********** PROCESS HTTP SERVER INPUTS FROM FORMS **********
//***********************************************************
//***********************************************************
//This optional function is called each time an input value is received with a GET or POST request.
//
//input_name
//	Pointer to a null terminated string containing the input name sent by the client (i.e. the name of the form item in your HTML page)
//input_value
//	Pointer to a null terminated string containing the value returned for this item
//requested_filename
//	Pointer to a null terminated string containing the filename that will be returned to the client after the driver has finished reading all of the input
//	data.  Your application may alter this if desired (max length = HTTP_MAX_FILENAME_LENGTH).  Can be ignored if you wish.
//requested_filename_extension
//	Pointer to 3 byte string containing the filename extension.  Your application may alter this if desired or it can be ignored.
//tcp_socket_number
//	Included in case it is helpful for your application to identify a user (e.g. by their mac or IP address, tcp_socket[tcp_socket_number].remote_device_info.ip_address)
//	Can be ignored if you wish.
void process_http_inputs (BYTE *input_name, BYTE *input_value, BYTE *requested_filename, BYTE *requested_filename_extension, BYTE tcp_socket_number);


//*****************************************************
//***** PROCESS MULTIPART HEADER FOR NEXT SECTION *****
//*****************************************************
//This function is called for each header found for a new multipart section, of a multipart form post
//It will be called 1 or more times, and this signifies that when process_http_multipart_form_data is next called it will
//be with the data for this new section of the multipart message (i.e. any call to this function means your about to receive
//data for a new multipart section, so reset whatever your application needs to reset to start dealing with the new data).
//The following are the possible values that this function can be called with (always lower case):-
//	content-disposition		Value will be 'form-data' or 'file'
//	name					Name of the corresponding form control
//	filename				Name of the file when content-disposition = file (note that the client is not requried to
//							provide this, but usually will).  May or may not include full path depending on browser.
//							If its important to you to read filename with potentially long paths ensure you set
//							HTTP_MAX_POST_LINE_LENGTH to a value that not cause the end to be cropped off long paths.
//	content-type			Value dependant on the content.  e.g. text/plain, image/gif, etc.
//							If not present then you must assume content-type = text/plain; charset=us-ascii
//
//input_name
//	Pointer to the null termianted header name string
//input_value
//	Pointer to the null termianted value string (converted to lowercase)
//requested_filename
//	Pointer to the null termianted string containing the filename, in case its useful
//requested_filename_extension
//	Pointer to the 3 byte string containing the filename extension, in case its useful
//tcp_socket_number
//	Included in case it is helpful for your application to identify a user (e.g. by their mac or IP address, tcp_socket[tcp_socket_number].remote_device_info.ip_address)

void process_http_multipart_form_header (const BYTE *input_name, BYTE *input_value, BYTE *requested_filename, BYTE *requested_filename_extension, BYTE tcp_socket_number);

//*******************************************************
//***** RECEIVE EACH BYTE OF THIS MULTIPART SECTION *****
//*******************************************************
//This function is called with each decoded byte in turn of a multipart section.  The data you get
//here is the same as the data submitted by the user.
//process_http_multipart_form_header() is called one of more times before this function starts getting
//called for each section.
void process_http_multipart_form_data (BYTE data);


//********************************************
//***** END OF MULTIPART SECTION REACHED *****
//********************************************
//Do any processing of last data block that may be required.
//This function is called after the last byte has been received for a multipart section to allow you to carry out any
//operations with the data just recevied before the next multipart section is started or the end of the form post.
void process_http_multipart_form_last_section_done (void);

#endif // APMAIN_H
