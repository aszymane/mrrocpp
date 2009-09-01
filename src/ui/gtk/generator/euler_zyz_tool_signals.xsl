<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
xyz_euler_zyz_tool window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.xyz_euler_zyz_tool.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="xyz_euler_zyz_tool" select="xyz_euler_zyz_tool"/>
<xsl:text>

extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz_tool (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.arrow">
    		<xsl:with-param name="xyz_euler_zyz_tool" select="$xyz_euler_zyz_tool"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>	
 	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz_tool (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.read.1">
    		<xsl:with-param name="xyz_euler_zyz_tool" select="$xyz_euler_zyz_tool"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
 		
		if (robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text></xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
			if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->read_tool_xyz_euler_zyz(tool_vector_e))) // Odczyt polozenia walow silnikow
					fprintf(stderr, "read euler_zyz_tool failed\n");
					
</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.read.2">
    				<xsl:with-param name="xyz_euler_zyz_tool" select="$xyz_euler_zyz_tool"/>
					<xsl:with-param name="name" select="$name"/>
					<xsl:with-param name="i" select="1"/>
 				</xsl:call-template><xsl:text>				
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout &lt;&lt; "Robot is not synchronized" &lt;&lt; std::endl;
			}
		}
	
	}
	
	void on_set_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz_tool (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.set.1">
    		<xsl:with-param name="xyz_euler_zyz_tool" select="$xyz_euler_zyz_tool"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>    

		if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised)
		{
	</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.set.2">
    		<xsl:with-param name="xyz_euler_zyz_tool" select="$xyz_euler_zyz_tool"/>
			<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>    
			
			robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->set_tool_xyz_euler_zyz(tool_vector_e);
		}
		on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz_tool (button, userdata);

	}
}
</xsl:text>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.set.1">
<xsl:param name="xyz_euler_zyz_tool"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
	<xsl:text>	GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
 	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
          <xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.set.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz_tool">
                  <xsl:value-of select="$xyz_euler_zyz_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.read.1">
<xsl:param name="xyz_euler_zyz_tool"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
	<xsl:text>	GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
          <xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.read.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz_tool">
                  <xsl:value-of select="$xyz_euler_zyz_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.read.2">
<xsl:param name="xyz_euler_zyz_tool"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
	<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);		
</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
          <xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.read.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz_tool">
                  <xsl:value-of select="$xyz_euler_zyz_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.set.2">
<xsl:param name="xyz_euler_zyz_tool"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
	<xsl:text>			tool_vector_e[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text>);
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
          <xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.set.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz_tool">
                  <xsl:value-of select="$xyz_euler_zyz_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 axis ts handling signals .cc repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.arrow">
<xsl:param name="xyz_euler_zyz_tool"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_tool_</xsl:text><xsl:value-of select="$name" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz_tool">
          <xsl:call-template name="irp6.xyz_euler_zyz_tool.repeat.signals.cc.arrow">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz_tool">
                  <xsl:value-of select="$xyz_euler_zyz_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

</xsl:stylesheet>
