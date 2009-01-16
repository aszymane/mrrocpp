
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6m_axis_xyz_uimodule.h"


edp_irp6m_axis_xyz::edp_irp6m_axis_xyz(ui_config_entry &entry) 
{
}

static edp_irp6m_axis_xyz *axis_xyz_mechatronika;


extern "C"
{
	void on_arrow_button_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla mechatronika axis_xyz" << std::endl;
	}
	
	void on_read_button_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla mechatronika axis_xyz" << std::endl;
	}
	
	void on_execute_button_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Execute move dla mechatronika axis_xyz" << std::endl;
	}
	
	void on_export_button_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Export dla mechatronika axis_xyz" << std::endl;
	}
	
	void on_import_button_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Import dla mechatronika axis_xyz" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		axis_xyz_mechatronika = new edp_irp6m_axis_xyz(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (axis_xyz_mechatronika) 
		{
			delete axis_xyz_mechatronika;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
	

	void on_button1_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button1 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button2_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button2 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button3_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button3 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button4_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button4 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button5_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button5 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button6_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button6 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button7_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button7 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button8_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button8 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button9_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button9 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button10_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button10 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button11_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button11 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button12_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button12 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button13_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button13 dla mechatronika axis_xyz" << std::endl;
	}
    

	void on_button14_clicked_mechatronika_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button14 dla mechatronika axis_xyz" << std::endl;
	}
    

}
