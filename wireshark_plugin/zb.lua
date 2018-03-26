do

	------------------------------------------------------------------------
	-- based on:
	-- "Dir (objects introspection like Python's dir) - Lua"
	-- http://snipplr.com/view/13085/
	-- (added iteration through getmetatable of userdata, and recursive call)
	-- make a global function here (in case it's needed in requires)
	--- Returns string representation of object obj
	-- @return String representation of obj
	------------------------------------------------------------------------
	function dir(obj,level)
	  local s,t = '', type(obj)

	  level = level or ' '

	  if (t=='nil') or (t=='boolean') or (t=='number') or (t=='string') then
		s = tostring(obj)
		if t=='string' then
		  s = '"' .. s .. '"'
		end
	  elseif t=='function' then s='function'
	  elseif t=='userdata' then
		s='userdata'
		for n,v in pairs(getmetatable(obj)) do  s = s .. " (" .. n .. "," .. dir(v) .. ")" end
	  elseif t=='thread' then s='thread'
	  elseif t=='table' then
		s = '{'
		for k,v in pairs(obj) do
		  local k_str = tostring(k)
		  if type(k)=='string' then
		    k_str = '["' .. k_str .. '"]'
		  end
		  s = s .. k_str .. ' = ' .. dir(v,level .. level) .. ', '
		end
		if s:len()>1 then
			s = string.sub(s, 1, -3)
		end
		s = s .. '}'
	  end
	  return s
	end

	local dprint = function(...)
		print(table.concat({"Lua:", ...}," "))
		local log = assert(io.open("lua.log", "a"))
		log:write(table.concat({"Lua:", ...}," ") .. "\n")
		log:close()
	end

	local default_settings =
	{
		comport  = '\\\\.\\pipe\\wiresharkTx',		-- This is the default for Windows
		channel  = 20
	}

	dprint("========================================")
	dprint("Started lua script on " .. os.date())
	dprint("========================================")

	local bHaveParamPort = false
	local bHaveParamChan = false

	dprint("Looking for environment variables")
	local ecom = os.getenv("ZBL_COMPORT")
	if(ecom~=nil)then
		default_settings["comport"] = ecom
		bHaveParamPort = true
		dprint("	Found environment COM port " .. ecom)
	end
	local echn = os.getenv("ZBL_CHANNEL")
	if(echn~=nil)then
		if tonumber(echn) and tonumber(echn)>=11 and tonumber(echn)<=26 then
			default_settings["channel"] = tonumber(echn)
			bHaveParamChan = true
			dprint("	Found environment Channel " .. echn)
		else
			info("	Unusable environment variable ZB_CHANNEL '"..echn.."' value must be a number between 11 and 26")
			dprint("	Unusable environment variable ZB_CHANNEL '"..echn.."' value must be a number between 11 and 26")
		end
	end
	dprint("Parsing command line parameters")
	local args={...} -- get passed-in args
	if args and #args > 0 then
		for _, arg in ipairs(args) do
		    local name, value = arg:match("(.+)=(.+)")
		    if name and value then
				dprint("	name="..name.."; value="..value)
				if name=="comport" then
					default_settings["comport"] = value
					dprint("	Found argument COM port " .. value)
					bHaveParamPort = true
				elseif name=="channel" then
					if tonumber(value) and tonumber(value)>=11 and tonumber(value)<=26 then
						default_settings["channel"] = tonumber(value)
						bHaveParamChan = true
						dprint("	Found argument Channel " .. value)
					else
						error("	commandline argument '"..name.."' value must be a number between 11 and 26")
					end
				else
					error("	unknow commandline argument '"..name.."'")
				end
		    else
		        error("	invalid commandline argument syntax")
		    end
		end
	end

--	local p_zbparams = Proto("zbparams","ZBParams");
	local p_zbparams104 = Proto("zbparams1","ZBParams1");
	local p_zbparams127 = Proto("zbparams2","ZBParams2");
	local f_channum1  = ProtoField.uint8("zbparams1.channum", "Channel", base.DEC)
	local f_chanfrq1  = ProtoField.float("zbparams1.chanfrq", "Frequency")
	local f_channum2  = ProtoField.uint8("zbparams2.channum", "Channel", base.DEC)
	local f_chanfrq2  = ProtoField.float("zbparams2.chanfrq", "Frequency")
	local f_ackref1   = ProtoField.framenum("zbparams1.acknum", "Acknowledge frame", base.NONE, frametype.ACK)
	local f_duration1 = ProtoField.relative_time("zbparams1.duration", "Duration of the frame", "Time it took to transmit the frame on the air, from the preamble to ...")
	local f_acktime   = ProtoField.relative_time("zbparams1.acktime", "Time since acked frame", "Time from the end of the AR frame to the start of the ack frame")

	local f_wpan             = Field.new("wpan")
	local f_wpan_fcf         = Field.new("wpan.fcf")
	local f_wpan_seq_no      = Field.new("wpan.seq_no")
	local f_frame_time_epoch = Field.new("frame.time_epoch")
	local f_frame_number     = Field.new("frame.number")
	local f_frame_len        = Field.new("frame.len")
	
	p_zbparams104.fields = { f_channum1, f_chanfrq1, f_ackref1, f_duration1, f_acktime }
	p_zbparams127.fields = { f_channum2, f_chanfrq2 }

	local data_dis = Dissector.get("data")

	function do_dissect(buf,pkt,subtree)
		pkt.cols.protocol = "zbparams"
		pkt.cols.src = "JN516x"
		pkt.cols.dst = "PC"

		local ftyp = buf(2,1):uint()
		if(ftyp==0)then
			local chan = buf(3,1):uint()
			local freq = buf(4,4):float()
			pkt.cols.info:append("Channel=" .. chan .. " (" .. string.format("%.3f", freq/1000000) .. " GHz)")
			subtree:add(f_channum1,buf:range(3,1))
			subtree:add(f_chanfrq1,buf:range(4,4)):set_text(string.format("Frequency: %.3f GHz", freq/1000000))
--			subtree:add(f_channum1,chan)
--			subtree:add(f_chanfrq1,freq/1000000):append_text(" GHz")
		elseif(ftyp==1)then
			pkt.cols.info = "Syntax error, could not interpret command."
		end
	end

-- Dissection routine
    function p_zbparams104.dissector(buf,pkt,root)
----		dprint("[0:2]=" .. buf(0,2):uint())
		set_color_filter_slot(4, "zbee_zcl")				-- Purple 2
		set_color_filter_slot(7, "zbee_nwk.cmd.id == 0x08")		-- Green  3 - Link Status
		if (buf:len() < 2) or (buf(0,2):uint() ~= 0x0700) then
			orig104:call(buf,pkt,root)
			local fcf  = f_wpan_fcf().value
			local fseq = f_wpan_seq_no().value
			local flen = f_frame_len().value
			local fnum = f_frame_number().value
--			local fnum = pkt.number
			local ftime = f_frame_time_epoch().value
			local wpan = f_wpan()
			if not packets[fnum] then
				packets[fnum] = {fseq, fcf, fnum, ftime}
--				packets[fnum] = {fseq, fcf, fnum}
			end
----			dprint(dir(fcf))
----			dprint("fcf="..tostring(fcf))
			if(fcf==0x0002)then
				local idx = fnum - 1
				local found = false
				dprint("===")
				while idx > 0 do
--					dprint(table.concat(packets[idx]," "))
--					dprint(packets[idx][1])
					if((packets[idx][1]==fseq))then
						found = true
						break
					end
					idx = idx - 1
				end
				dprint("===")
				if(found) then
					local fack = root:add(f_ackref1,idx):set_generated()
					fack:add(f_acktime, ftime-packets[idx][4]):set_generated()
				else
					root:add("Corresponding frame not found"):set_generated()
				end
----				dprint("fcf="..fcf)
			end
--			wpan:add(f_duration1, NSTime(flen/4./62500., flen/4./62500.*10^9)):set_generated()
			root:add(f_duration1, NSTime(flen/4./62500., flen/4./62500.*10^9)):set_generated()
		else
			local pktlen = buf:reported_length_remaining()
			local subtree = root:add(p_zbparams104,buf:range(0,pktlen))
			do_dissect(buf,pkt,subtree)
		end
--		dprint(dir(pkt))
--		dprint(tostring(pkt))
--		dprint(tostring(pkt.cols))
--		dprint(dir(pkt.cols))
--		dprint(dir(pkt.cols[0]))
--		dprint(pkt.cols:len())
--		dprint(pkt["in_error_pkt"])
--		dprint(pkt.dl_dst)
--		dprint(pkt.dl_dst())
--		"] = function, ["delta_ts"] = function, ["visited"] = function, ["len"] = function, ["net_src"] = function, ["in_error_pkt"] = function, ["match_uint"] = function, ["circuit_id"] = function, ["match_string"] = function, ["delta_dis_ts"] = function, ["fragmented"] = function, ["can_desegment"] = function, ["number"] = function, ["port_type"] = function, ["desegment_len"] = function, ["columns"] = function, ["dst"] = function, ["desegment_offset"] = function, ["hi"] = function, ["rel_ts"] = function, ["dl_src"] = function, ["cols"] = function, ["curr_proto"] = function, ["match"] = function, ["dst_port"] = function, ["caplen"] = function, ["src_port"] = function, ["net_dst"] = function, ["abs_ts"] = function, ["lo"] = function, ["src"] = function, ["private"] = function}) (__tostring,function) (__gc,function) (__setters,{["dl_dst"] = function, ["conversation"] = function, ["src_port"] = function, ["net_src"] = function, ["circuit_id"] = function, ["dst_port"] = function, ["dst"] = function, ["desegment_offset"] = function, ["net_dst"] = function, ["desegment_len"] = function, ["can_desegment"] = function, ["src"] = function, ["dl_src"]
    end

    function p_zbparams127.dissector(buf,pkt,root)

		set_color_filter_slot(4, "zbee_zcl")				-- Purple 2
		set_color_filter_slot(7, "zbee_nwk.cmd.id == 0x08")		-- Green  3 - Link Status
		if (buf:len() < 2) or (buf(0,2):uint() ~= 0x0700) then
			orig127:call(buf,pkt,root)
		else
			local pktlen = buf:reported_length_remaining()
			local subtree = root:add(p_zbparams127,buf:range(0, pktlen))
			do_dissect(buf,pkt,subtree)
		end
		dprint(dir(pkt))
    end

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
--		dprint("p_zbparams104 prefs_changed called: channel = " .. p_zbparams104.prefs.channel .. ", serial port = " .. p_zbparams104.prefs.comport)
		if(not bHaveParamPort)then
			default_settings.comport  = p_zbparams104.prefs.comport
			dprint("Using saved port " .. default_settings.comport)
		end
		if(not bHaveParamChan)then
			default_settings.channel  = p_zbparams104.prefs.channel
			dprint("Using saved channel " .. default_settings.channel)
			local portname = default_settings.comport
			local channel  = default_settings.channel
			local com = assert(io.open(portname, "w"))
			com:write("C:" .. channel .. "\n")
			com:close()
		end
	end

-- Initialization routine
	function p_zbparams104.init()
		packets = {}
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

			if portname == "" then 
				portname = default_settings.comport
			else
				default_settings.comport = portname
			end
			if channel  == "" then
				channel = default_settings.channel
			else
				default_settings.channel = channel
			end
			dprint("About to send C:" .. channel .. " on " .. portname)
			local com = assert(io.open(portname, "w"))
			com:write("C:" .. channel .. "\n")
			com:close()
		end

		local portname = default_settings.comport
		local channel  = default_settings.channel
		new_dialog("ZB Config",dialog_func,"Serial port (blank is " .. portname .. ")","Channel (blank is " .. channel .. ")")

	end

	local function zbinit()
		local portname = default_settings.comport
		local channel  = default_settings.channel
		local com = assert(io.open(portname, "w"))
		dprint("About to send INI:" .. channel .. " on " .. portname)
		com:write("INI:" .. channel .. "\n")
--		com:write("STA\n")
--		dprint("About to send C:" .. channel .. " on " .. portname)
--		com:write("C:" .. channel .. "\n")
		com:close()
	end
	local function zbstart()
		local portname = default_settings.comport
		local channel  = default_settings.channel
		local com = assert(io.open(portname, "w"))
		dprint("About to send STA:" .. channel .. " on " .. portname)
		com:write("STA:".. channel .. "\n")
--		dprint("About to send C:" .. channel .. " on " .. portname)
--		com:write("C:" .. channel .. "\n")
		com:close()
	end
	local function zbstop()
		local portname = default_settings.comport
		local com = assert(io.open(portname, "w"))
		dprint("About to send STO on " .. portname)
		com:write("STO\n")
		com:close()
	end
	local function zbtest()
		local portname = default_settings.comport
		local com = assert(io.open(portname, "w"))
		dprint("About to send TST on " .. portname)
		com:write("TST\n")
		com:close()
	end
	local function zbbrq()
		local portname = default_settings.comport
		local com = assert(io.open(portname, "w"))
		dprint("About to send BRQ on " .. portname)
		com:write("BRQ\n")
		com:close()
	end
	local function zbsbq()
		local portname = default_settings.comport
		local com = assert(io.open(portname, "w"))
		dprint("About to send SBQ on " .. portname)
		com:write("SBQ\n")
		com:close()
	end
	local function zb1Mbps()
		local portname = default_settings.comport
		local com = assert(io.open(portname, "w"))
		dprint("About to send BRD:1000000 on " .. portname)
		com:write("BRD:1000000\n")
		com:close()
	end

--	register_menu("ZB/0. Init",zbinit,MENU_TOOLS_UNSORTED)
	register_menu("ZB/1. Start",zbstart,MENU_TOOLS_UNSORTED)
	register_menu("ZB/2. Stop",zbstop,MENU_TOOLS_UNSORTED)
	register_menu("ZB/3. Options",dialog_menu,MENU_TOOLS_UNSORTED)
	register_menu("ZB/8. Set 1Mbps",zb1Mbps,MENU_TOOLS_UNSORTED)
	register_menu("ZB/9. Test",zbtest,MENU_TOOLS_UNSORTED)
	register_menu("ZB/A. Beacon Request",zbbrq,MENU_TOOLS_UNSORTED)
	register_menu("ZB/B. Beacon",zbsbq,MENU_TOOLS_UNSORTED)
end

