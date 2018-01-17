do
--	if gui_enabled() then
--	   local splash = TextWindow.new("Hello!");
--	   splash:set("This time wireshark has been enhanced with a useless feature.\n")
--	   splash:append("Go to 'Tools->Lua Dialog Test' and check it out!")
--	end
--local chans_freqs_enum = { {"2.405 GHz"},
--                            {"2.410 GHz"},
--                            {"2.415 GHz"},
--                            {"2.420 GHz\0"},
--                            {"2.425 GHz\0"},
--                            {"2.430 GHz\0"},
--                            {"2.435 GHz\0"},
--                            {"2.440 GHz\0"},
--                            {"2.445 GHz\0"},
--                            {"2.450 GHz\0"},
 --                           {"2.455 GHz\0"},
--                            {"2.460 GHz\0"},
--                            {"2.465 GHz\0"},
--                            {"2.470 GHz\0"},
--                            {"2.475 GHz\0"},
--                            {"2.480 GHz\0"}};

	local dprint = function(...)
		print(table.concat({"Lua:", ...}," "))
	end

--	local p_zbparams = Proto("zbparams","ZBParams");
	local p_zbparams104 = Proto("zbparams1","ZBParams1");
	local p_zbparams127 = Proto("zbparams2","ZBParams2");
	local f_channum1  = ProtoField.uint8("zbparams1.channum", "Channel", base.DEC)
	local f_chanfrq1  = ProtoField.float("zbparams1.chanfrq", "Frequency")
	local f_channum2  = ProtoField.uint8("zbparams2.channum", "Channel", base.DEC)
	local f_chanfrq2  = ProtoField.float("zbparams2.chanfrq", "Frequency")
	
	p_zbparams104.fields = { f_channum1, f_chanfrq1 }
	p_zbparams127.fields = { f_channum2, f_chanfrq2 }

	local data_dis = Dissector.get("data")

	function do_dissect(buf,pkt,subtree)
		pkt.cols.protocol = "zbparams"
		pkt.cols.src = "JN516x"
		pkt.cols.dst = "PC"

		local chan = buf(3,1):uint()
		local freq = buf(4,4):float()
		pkt.cols.info:append("Channel=" .. chan .. " (" .. string.format("%.3f", freq/1000000) .. " GHz)")
		subtree:add(f_channum1,buf:range(3,1))
		subtree:add(f_chanfrq1,buf:range(4,4)):set_text(string.format("Frequency: %.3f GHz", freq/1000000))
--		subtree:add(f_channum1,chan)
--		subtree:add(f_chanfrq1,freq/1000000):append_text("GHz")
	end

-- Dissection routine
    function p_zbparams104.dissector(buf,pkt,root)
--		dprint("[0:2]=" .. buf(0,2):uint())
		if (buf:len() < 2) or (buf(0,2):uint() ~= 0x0700) then
			orig104:call(buf,pkt,root)
		else
			local pktlen = buf:reported_length_remaining()
			local subtree = root:add(p_zbparams104,buf:range(0,pktlen))
			do_dissect(buf,pkt,subtree)
		end

--		local ptr = 0
--		while(ptr<buf:len()) do
--			ptr = ptr + do_frame(buf(ptr,buf:len()-ptr):tvb(),pkt,root)
--		end

--                local proto_id = buf(0,1):uint()

--                local dissector = protos[proto_id]

--                if dissector ~= nil then
--                        dissector:call(buf(2):tvb(),pkt,root)
--                elseif proto_id < 2 then
--                        tbl:add(f_text,buf(2))
--                        -- pkt.cols.info:set(buf(2,buf:len() - 3):string())
--                else
--                        data_dis:call(buf(2):tvb(),pkt,root)
--                end

    end

    function p_zbparams127.dissector(buf,pkt,root)
--		dprint("[0:2]=" .. buf(0,2):uint())
		if (buf:len() < 2) or (buf(0,2):uint() ~= 0x0700) then
			orig127:call(buf,pkt,root)
		else
			local pktlen = buf:reported_length_remaining()
			local subtree = root:add(p_zbparams127,buf:range(0, pktlen))
			do_dissect(buf,pkt,subtree)
		end
    end

	local default_settings =
	{
		comport  = "/dev/ttyUSB1",
		channel  = 20
	}

	local channel_pref_enum = {
		{  1,  "Channel 11 (2.405GHz)", 11 },
		{  2,  "Channel 12 (2.410GHz)", 12  },
		{  3,  "Channel 13 (2.415GHz)", 13  },
		{  4,  "Channel 14 (2.420GHz)", 14  },
		{  5,  "Channel 15 (2.425GHz)", 15  },
		{  6,  "Channel 16 (2.430GHz)", 16  },
		{  7,  "Channel 17 (2.435GHz)", 17  },
		{  8,  "Channel 18 (2.440GHz)", 18  },
		{  9,  "Channel 19 (2.445GHz)", 19  },
		{ 10,  "Channel 20 (2.450GHz)", 20  },
		{ 11,  "Channel 21 (2.455GHz)", 21  },
		{ 12,  "Channel 22 (2.460GHz)", 22  },
		{ 13,  "Channel 23 (2.465GHz)", 23  },
		{ 14,  "Channel 24 (2.470GHz)", 24  },
		{ 15,  "Channel 25 (2.475GHz)", 25  },
		{ 16,  "Channel 26 (2.480GHz)", 25  },
	}
-- Parameters
	p_zbparams104.prefs.comport = Pref.string("Serial port", default_settings.comport, "Serial port used to send commands")
	p_zbparams104.prefs.channel = Pref.enum("Channel", default_settings.channel, "Zigbee channel to listen on", channel_pref_enum)
	function p_zbparams104.prefs_changed()
	    dprint("p_zbparams104 prefs_changed called: channel = " .. p_zbparams104.prefs.channel .. ", serial port = " .. p_zbparams104.prefs.comport)
	    default_settings.channel  = p_zbparams104.prefs.channel
		default_settings.comport  = p_zbparams104.prefs.comport
	end

-- Initialization routine
	function p_zbparams104.init()
	end
	function p_zbparams127.init()
	end
-- Register dissector
	local wtap_encap_table = DissectorTable.get("wtap_encap")
	orig104 = wtap_encap_table:get_dissector(104)
	orig127 = wtap_encap_table:get_dissector(127)
	wtap_encap_table:add(104, p_zbparams104)
	wtap_encap_table:add(127, p_zbparams127)
--	local wpan_panid_table = DissectorTable.get("wpan.panid")
--	wpan_panid_table:add(3, p_zbparams)

	local function dialog_menu()
		local function dialog_func(portname,channel)
--		    local win = TextWindow.new("ZigBee configuration");
--		    win:set(person)
--		    win:append(" with " .. eyes .." eyes and")
--		    win:append(" " .. hair .. " hair.");
			
--			local portname = "/dev/ttyUSB0"
			if portname == "" then 
				portname = default_settings.comport
			else
				default_settings.comport = portname
			end
			if channel  == "" then
				default_settings.channel = channel
			end
			dprint("About to open file...")
			local com = assert(io.open(portname, "w"))
			com:write("C:" .. channel .. "\n")
			com:close()
			dprint("Closed file...")

		end

		local portname = default_settings.comport
		local channel  = default_settings.channel
		new_dialog("ZB Config",dialog_func,"Serial port (blank is " .. portname .. ")","Channel (blank is " .. channel .. ")")
--		dprint("About to open file...")
--		local com = assert(io.open(portname, "w"))
--		com:write("C:25\n")
--		com:close()
--		dprint("Closed file...")
	end

	local function zbstart()
		local portname = default_settings.comport
		local com = assert(io.open(portname, "w"))
		com:write("STA\n")
		com:close()
	end
	local function zbstop()
		local portname = default_settings.comport
		local com = assert(io.open(portname, "w"))
		com:write("STO\n")
		com:close()
	end
	local function zbtest()
		local portname = default_settings.comport
		local com = assert(io.open(portname, "w"))
		com:write("TST\n")
		com:close()
	end

	register_menu("ZB/ZB Options",dialog_menu,MENU_TOOLS_UNSORTED)
	register_menu("ZB/ZB Start",zbstart,MENU_TOOLS_UNSORTED)
	register_menu("ZB/ZB Stop",zbstop,MENU_TOOLS_UNSORTED)
	register_menu("ZB/ZB Test",zbtest,MENU_TOOLS_UNSORTED)
end

------------------------------------------------------------------------
-- This Example will add a menu "Lua Dialog Test" under the Tools menu, 
-- which when selected will pop a dialog prompting the user for input 
-- that when accepted will pop a window with a result.

--if gui_enabled() then
--   local splash = TextWindow.new("Hello!");
--   splash:set("This time wireshark has been enhanced with a useless feature.\n")
--   splash:append("Go to 'Tools->Lua Dialog Test' and check it out!")
--end
--local function dialog_menu()
--    local function dialog_func(person,eyes,hair)
--        local win = TextWindow.new("The Person");
--        win:set(person)
--        win:append(" with " .. eyes .." eyes and")
--        win:append(" " .. hair .. " hair.");
--    end

--    new_dialog("Dialog Test",dialog_func,"A Person","Eyes","Hair")
--end

-- optional 3rd parameter to register_menu. 
-- See http://www.wireshark.org/docs/wsug_html_chunked/wsluarm_modules.html 
-- If omitted, defaults to MENU_STAT_GENERIC. Other options include:
-- MENU_STAT_UNSORTED (Statistics), 
-- MENU_STAT_GENERIC (Statistics, first section), 
-- MENU_STAT_CONVERSATION (Statistics/Conversation List), 
-- MENU_STAT_ENDPOINT (Statistics/Endpoint List), 
-- MENU_STAT_RESPONSE (Statistics/Service Response Time), 
-- MENU_STAT_TELEPHONY (Telephony), 
-- MENU_ANALYZE_UNSORTED (Analyze), 
-- MENU_ANALYZE_CONVERSATION (Analyze/Conversation Filter), 
-- MENU_TOOLS_UNSORTED (Tools)

--register_menu("Lua Dialog Test",dialog_menu,MENU_TOOLS_UNSORTED)
