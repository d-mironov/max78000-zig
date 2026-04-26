const std = @import("std");

const Allocator = std.mem.Allocator;

const ElementRange = struct {
    start: usize,
    end: usize,
};

const ElementBounds = struct {
    name: []const u8,
    open_end: usize,
    close_start: usize,
    end: usize,
    self_closing: bool,
};

const ParsedTag = struct {
    name: []const u8,
    end: usize,
    is_closing: bool,
    is_self_closing: bool,
};

pub fn main() !void {
    var gpa_state = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa_state.deinit();
    const gpa = gpa_state.allocator();

    var args = try std.process.argsWithAllocator(gpa);
    defer args.deinit();

    _ = args.next();
    const input_path = args.next() orelse return error.MissingInputPath;

    const output_flag = args.next() orelse return error.MissingOutputFlag;
    if (!std.mem.eql(u8, output_flag, "--output-path")) return error.InvalidArguments;

    const output_path = args.next() orelse return error.MissingOutputPath;
    if (args.next() != null) return error.InvalidArguments;

    const input = try readWholeFileAlloc(gpa, input_path);
    defer gpa.free(input);

    const output = try expandMax78000Svd(gpa, input);
    defer gpa.free(output);

    try validateExpandedMax78000Svd(input, output);
    try writeWholeFile(output_path, output);
}

fn readWholeFileAlloc(allocator: Allocator, path: []const u8) ![]u8 {
    const file = try std.fs.cwd().openFile(path, .{});
    defer file.close();
    const stat = try file.stat();
    return try file.readToEndAlloc(allocator, stat.size + 1);
}

fn writeWholeFile(path: []const u8, data: []const u8) !void {
    var file = try std.fs.cwd().createFile(path, .{});
    defer file.close();
    try file.writeAll(data);
}

pub fn expandMax78000Svd(allocator: Allocator, input: []const u8) ![]u8 {
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();
    const arena = arena_state.allocator();

    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(arena);

    var cursor: usize = 0;
    while (true) {
        const registers_tag = "<registers>";
        const registers_open = std.mem.indexOfPos(u8, input, cursor, registers_tag) orelse break;
        const registers_content_start = registers_open + registers_tag.len;
        const registers_close = std.mem.indexOfPos(u8, input, registers_content_start, "</registers>") orelse return error.InvalidXml;

        try out.appendSlice(arena, input[cursor..registers_content_start]);

        const expanded_content = try expandRegistersContent(arena, input[registers_content_start..registers_close]);
        try out.appendSlice(arena, expanded_content);

        cursor = registers_close;
    }

    try out.appendSlice(arena, input[cursor..]);

    return allocator.dupe(u8, out.items);
}

fn validateExpandedMax78000Svd(input: []const u8, output: []const u8) !void {
    const peripheral_derived_in = countOccurrences(input, "<peripheral derivedFrom=\"");
    const peripheral_derived_out = countOccurrences(output, "<peripheral derivedFrom=\"");

    if (peripheral_derived_in != peripheral_derived_out) return error.PeripheralDerivedFromCountChanged;
    if (countOccurrences(output, "<register derivedFrom=\"") != 0) return error.UnexpandedRegisterDerivedFrom;
    if (countOccurrences(output, "<field derivedFrom=\"") != 0) return error.UnexpandedFieldDerivedFrom;

    try expectContains(output, "<name>LPWKST1</name>");
    try expectContains(output, "<name>LPWKEN1</name>");
    try expectContains(output, "<name>BUCK_OUT_READY</name>");
    try expectContains(output, "<name>BUCKOUTRDYB</name>");
    try expectContains(output, "<name>CH1</name>");
    try expectContains(output, "<name>CH2</name>");
    try expectContains(output, "<name>CH3</name>");

    try expectWithinWindow(output, "<name>LPWKST1</name>", "<name>WAKEST</name>", 1800);
    try expectWithinWindow(output, "<name>LPWKEN1</name>", "<name>WAKEEN</name>", 1800);
    try expectWithinWindow(output, "<name>BUCK_OUT_READY</name>", "<name>BUCKOUTRDYB</name>", 2200);
}

fn countOccurrences(haystack: []const u8, needle: []const u8) usize {
    var count: usize = 0;
    var cursor: usize = 0;
    while (std.mem.indexOfPos(u8, haystack, cursor, needle)) |idx| {
        count += 1;
        cursor = idx + needle.len;
    }
    return count;
}

fn expectContains(haystack: []const u8, needle: []const u8) !void {
    if (std.mem.indexOf(u8, haystack, needle) == null) return error.ValidationNeedleMissing;
}

fn expectWithinWindow(haystack: []const u8, anchor: []const u8, needle: []const u8, window: usize) !void {
    const anchor_pos = std.mem.indexOf(u8, haystack, anchor) orelse return error.ValidationAnchorMissing;
    const needle_pos = std.mem.indexOfPos(u8, haystack, anchor_pos, needle) orelse return error.ValidationNeedleMissingNearAnchor;
    if ((needle_pos - anchor_pos) > window) return error.ValidationNeedleTooFarFromAnchor;
}

fn expandRegistersContent(allocator: Allocator, content: []const u8) ![]const u8 {
    var out: std.ArrayList(u8) = .empty;

    var register_map = std.StringHashMap([]const u8).init(allocator);

    var cursor: usize = 0;
    while (try nextDirectChildByTag(content, cursor, "register")) |register_range| {
        try out.appendSlice(allocator, content[cursor..register_range.start]);

        const register_xml = content[register_range.start..register_range.end];
        const expanded_register = try expandRegister(allocator, register_xml, &register_map);

        const register_name = try getDirectChildText(expanded_register, "name");
        try register_map.put(register_name, expanded_register);

        try out.appendSlice(allocator, expanded_register);
        cursor = register_range.end;
    }

    try out.appendSlice(allocator, content[cursor..]);
    return out.items;
}

fn expandRegister(
    allocator: Allocator,
    register_xml: []const u8,
    register_map: *std.StringHashMap([]const u8),
) ![]const u8 {
    const base = if (try getAttribute(register_xml, "derivedFrom")) |source_name|
        register_map.get(source_name) orelse return error.MissingSourceRegister
    else
        register_xml;

    const merged = if (try getAttribute(register_xml, "derivedFrom") != null)
        try mergeElement(allocator, "register", base, register_xml)
    else
        try removeDerivedFromAttribute(allocator, register_xml);

    return try expandFieldsInRegister(allocator, merged);
}

fn expandFieldsInRegister(allocator: Allocator, register_xml: []const u8) ![]const u8 {
    const fields_range = (try findDirectChildRange(register_xml, "fields")) orelse return register_xml;
    const fields_xml = register_xml[fields_range.start..fields_range.end];

    const fields_bounds = try getElementBounds(fields_xml);
    const fields_content = fields_xml[fields_bounds.open_end..fields_bounds.close_start];

    var out_fields: std.ArrayList(u8) = .empty;
    defer out_fields.deinit(allocator);

    var field_map = std.StringHashMap([]const u8).init(allocator);

    var cursor: usize = 0;
    while (try nextDirectChildByTag(fields_content, cursor, "field")) |field_range| {
        try out_fields.appendSlice(allocator, fields_content[cursor..field_range.start]);

        const field_xml = fields_content[field_range.start..field_range.end];
        const expanded_field = try expandField(allocator, field_xml, &field_map);

        const field_name = try getDirectChildText(expanded_field, "name");
        try field_map.put(field_name, expanded_field);

        try out_fields.appendSlice(allocator, expanded_field);
        cursor = field_range.end;
    }

    try out_fields.appendSlice(allocator, fields_content[cursor..]);

    const new_fields_xml = try replaceInnerContent(allocator, fields_xml, out_fields.items);
    return try replaceRange(allocator, register_xml, fields_range.start, fields_range.end, new_fields_xml);
}

fn expandField(
    allocator: Allocator,
    field_xml: []const u8,
    field_map: *std.StringHashMap([]const u8),
) ![]const u8 {
    const source_name = try getAttribute(field_xml, "derivedFrom") orelse return try removeDerivedFromAttribute(allocator, field_xml);
    const source = field_map.get(source_name) orelse return error.MissingSourceField;
    return try mergeElement(allocator, "field", source, field_xml);
}

fn mergeElement(
    allocator: Allocator,
    comptime tag_name: []const u8,
    base_xml: []const u8,
    override_xml: []const u8,
) ![]const u8 {
    var merged = try removeDerivedFromAttribute(allocator, base_xml);

    var cursor: usize = 0;
    while (try nextDirectChildAny(override_xml, cursor)) |override_child_range| {
        const override_child = override_xml[override_child_range.start..override_child_range.end];
        const child_tag = try getElementName(override_child);

        if (std.mem.eql(u8, child_tag, tag_name)) {
            return error.InvalidXml;
        }

        if (try findDirectChildRange(merged, child_tag)) |base_child_range| {
            merged = try replaceRange(allocator, merged, base_child_range.start, base_child_range.end, override_child);
        } else {
            merged = try insertBeforeClose(allocator, merged, override_child);
        }

        cursor = override_child_range.end;
    }

    return try removeDerivedFromAttribute(allocator, merged);
}

fn replaceInnerContent(allocator: Allocator, element_xml: []const u8, new_inner: []const u8) ![]const u8 {
    const bounds = try getElementBounds(element_xml);
    if (bounds.self_closing) return error.InvalidXml;

    var out: std.ArrayList(u8) = .empty;

    try out.appendSlice(allocator, element_xml[0..bounds.open_end]);
    try out.appendSlice(allocator, new_inner);
    try out.appendSlice(allocator, element_xml[bounds.close_start..]);
    return out.items;
}

fn insertBeforeClose(allocator: Allocator, element_xml: []const u8, child_xml: []const u8) ![]const u8 {
    const bounds = try getElementBounds(element_xml);
    if (bounds.self_closing) return error.InvalidXml;

    var out: std.ArrayList(u8) = .empty;

    try out.appendSlice(allocator, element_xml[0..bounds.close_start]);
    try out.appendSlice(allocator, child_xml);
    try out.appendSlice(allocator, element_xml[bounds.close_start..]);
    return out.items;
}

fn replaceRange(allocator: Allocator, original: []const u8, start: usize, end: usize, replacement: []const u8) ![]const u8 {
    var out: std.ArrayList(u8) = .empty;

    try out.appendSlice(allocator, original[0..start]);
    try out.appendSlice(allocator, replacement);
    try out.appendSlice(allocator, original[end..]);
    return out.items;
}

fn findDirectChildRange(element_xml: []const u8, child_tag: []const u8) !?ElementRange {
    const bounds = try getElementBounds(element_xml);
    if (bounds.self_closing) return null;

    const inner = element_xml[bounds.open_end..bounds.close_start];
    const child = try nextDirectChildByTag(inner, 0, child_tag) orelse return null;

    return .{
        .start = bounds.open_end + child.start,
        .end = bounds.open_end + child.end,
    };
}

fn getDirectChildText(element_xml: []const u8, child_tag: []const u8) ![]const u8 {
    const range = (try findDirectChildRange(element_xml, child_tag)) orelse return error.MissingChild;
    const child_xml = element_xml[range.start..range.end];
    const child_bounds = try getElementBounds(child_xml);
    if (child_bounds.self_closing) return "";
    return std.mem.trim(u8, child_xml[child_bounds.open_end..child_bounds.close_start], " \t\r\n");
}

fn removeDerivedFromAttribute(allocator: Allocator, element_xml: []const u8) ![]const u8 {
    const open_end = try getOpeningTagEnd(element_xml);
    const open_tag = element_xml[0..open_end];

    const marker = "derivedFrom=\"";
    const marker_pos = std.mem.indexOf(u8, open_tag, marker) orelse return allocator.dupe(u8, element_xml);

    var remove_start = marker_pos;
    while (remove_start > 0 and std.ascii.isWhitespace(open_tag[remove_start - 1])) : (remove_start -= 1) {}

    var remove_end = marker_pos + marker.len;
    while (remove_end < open_tag.len and open_tag[remove_end] != '"') : (remove_end += 1) {}
    if (remove_end >= open_tag.len) return error.InvalidXml;
    remove_end += 1;

    var out: std.ArrayList(u8) = .empty;

    try out.appendSlice(allocator, open_tag[0..remove_start]);
    try out.appendSlice(allocator, open_tag[remove_end..]);
    try out.appendSlice(allocator, element_xml[open_end..]);

    return out.items;
}

fn getAttribute(element_xml: []const u8, attr_name: []const u8) !?[]const u8 {
    const open_end = try getOpeningTagEnd(element_xml);
    const open_tag = element_xml[0..open_end];

    var needle_buf: [128]u8 = undefined;
    const needle = try std.fmt.bufPrint(&needle_buf, "{s}=\"", .{attr_name});

    const start = std.mem.indexOf(u8, open_tag, needle) orelse return null;
    const value_start = start + needle.len;
    const value_end = std.mem.indexOfScalarPos(u8, open_tag, value_start, '"') orelse return error.InvalidXml;
    return open_tag[value_start..value_end];
}

fn getOpeningTagEnd(element_xml: []const u8) !usize {
    const lt = std.mem.indexOfScalar(u8, element_xml, '<') orelse return error.InvalidXml;
    if (lt != 0) return error.InvalidXml;

    var i: usize = 1;
    while (i < element_xml.len) : (i += 1) {
        if (element_xml[i] == '>') return i + 1;
    }
    return error.InvalidXml;
}

fn getElementName(element_xml: []const u8) ![]const u8 {
    const tag = try parseTag(element_xml, 0);
    if (tag.is_closing) return error.InvalidXml;
    return tag.name;
}

fn getElementBounds(element_xml: []const u8) !ElementBounds {
    const open_tag = try parseTag(element_xml, 0);
    if (open_tag.is_closing) return error.InvalidXml;

    if (open_tag.is_self_closing) {
        return .{
            .name = open_tag.name,
            .open_end = open_tag.end,
            .close_start = open_tag.end,
            .end = open_tag.end,
            .self_closing = true,
        };
    }

    var depth: usize = 1;
    var cursor: usize = open_tag.end;

    while (cursor < element_xml.len) {
        const lt = std.mem.indexOfScalarPos(u8, element_xml, cursor, '<') orelse return error.InvalidXml;

        if (std.mem.startsWith(u8, element_xml[lt..], "<!--")) {
            cursor = try skipComment(element_xml, lt);
            continue;
        }

        if (std.mem.startsWith(u8, element_xml[lt..], "<?")) {
            cursor = try skipProcessingInstruction(element_xml, lt);
            continue;
        }

        if (std.mem.startsWith(u8, element_xml[lt..], "<!")) {
            cursor = try skipDeclaration(element_xml, lt);
            continue;
        }

        const tag = try parseTag(element_xml, lt);
        if (std.mem.eql(u8, tag.name, open_tag.name)) {
            if (tag.is_closing) {
                depth -= 1;
                if (depth == 0) {
                    return .{
                        .name = open_tag.name,
                        .open_end = open_tag.end,
                        .close_start = lt,
                        .end = tag.end,
                        .self_closing = false,
                    };
                }
            } else if (!tag.is_self_closing) {
                depth += 1;
            }
        }

        cursor = tag.end;
    }

    return error.InvalidXml;
}

fn nextDirectChildByTag(content: []const u8, start_pos: usize, wanted_tag: []const u8) !?ElementRange {
    var depth: usize = 0;
    var cursor = start_pos;

    while (cursor < content.len) {
        const lt = std.mem.indexOfScalarPos(u8, content, cursor, '<') orelse return null;

        if (std.mem.startsWith(u8, content[lt..], "<!--")) {
            cursor = try skipComment(content, lt);
            continue;
        }

        if (std.mem.startsWith(u8, content[lt..], "<?")) {
            cursor = try skipProcessingInstruction(content, lt);
            continue;
        }

        if (std.mem.startsWith(u8, content[lt..], "<!")) {
            cursor = try skipDeclaration(content, lt);
            continue;
        }

        const tag = try parseTag(content, lt);

        if (tag.is_closing) {
            if (depth > 0) depth -= 1;
            cursor = tag.end;
            continue;
        }

        if (depth == 0 and std.mem.eql(u8, tag.name, wanted_tag)) {
            const bounds = try getElementBounds(content[lt..]);
            return .{
                .start = lt,
                .end = lt + bounds.end,
            };
        }

        if (!tag.is_self_closing) depth += 1;
        cursor = tag.end;
    }

    return null;
}

fn nextDirectChildAny(element_xml: []const u8, start_pos: usize) !?ElementRange {
    const parent_bounds = try getElementBounds(element_xml);
    if (parent_bounds.self_closing) return null;

    const inner = element_xml[parent_bounds.open_end..parent_bounds.close_start];

    const start_inner = if (start_pos <= parent_bounds.open_end) 0 else start_pos - parent_bounds.open_end;

    var depth: usize = 0;
    var cursor = start_inner;
    while (cursor < inner.len) {
        const lt = std.mem.indexOfScalarPos(u8, inner, cursor, '<') orelse return null;

        if (std.mem.startsWith(u8, inner[lt..], "<!--")) {
            cursor = try skipComment(inner, lt);
            continue;
        }

        if (std.mem.startsWith(u8, inner[lt..], "<?")) {
            cursor = try skipProcessingInstruction(inner, lt);
            continue;
        }

        if (std.mem.startsWith(u8, inner[lt..], "<!")) {
            cursor = try skipDeclaration(inner, lt);
            continue;
        }

        const tag = try parseTag(inner, lt);

        if (tag.is_closing) {
            if (depth > 0) depth -= 1;
            cursor = tag.end;
            continue;
        }

        if (depth == 0) {
            const bounds = try getElementBounds(inner[lt..]);
            return .{
                .start = parent_bounds.open_end + lt,
                .end = parent_bounds.open_end + lt + bounds.end,
            };
        }

        if (!tag.is_self_closing) depth += 1;
        cursor = tag.end;
    }

    return null;
}

fn parseTag(text: []const u8, start: usize) !ParsedTag {
    if (start >= text.len or text[start] != '<') return error.InvalidXml;

    var i = start + 1;
    var is_closing = false;
    if (i < text.len and text[i] == '/') {
        is_closing = true;
        i += 1;
    }

    while (i < text.len and std.ascii.isWhitespace(text[i])) : (i += 1) {}
    if (i >= text.len) return error.InvalidXml;

    const name_start = i;
    while (i < text.len and isTagNameChar(text[i])) : (i += 1) {}
    if (i == name_start) return error.InvalidXml;
    const name_end = i;

    const gt = std.mem.indexOfScalarPos(u8, text, i, '>') orelse return error.InvalidXml;

    var j = gt;
    while (j > start and std.ascii.isWhitespace(text[j - 1])) : (j -= 1) {}
    const is_self_closing = !is_closing and j > start and text[j - 1] == '/';

    return .{
        .name = text[name_start..name_end],
        .end = gt + 1,
        .is_closing = is_closing,
        .is_self_closing = is_self_closing,
    };
}

fn isTagNameChar(c: u8) bool {
    return std.ascii.isAlphanumeric(c) or c == '_' or c == '-' or c == ':';
}

fn skipComment(text: []const u8, start: usize) !usize {
    const end = std.mem.indexOfPos(u8, text, start + 4, "-->") orelse return error.InvalidXml;
    return end + 3;
}

fn skipProcessingInstruction(text: []const u8, start: usize) !usize {
    const end = std.mem.indexOfPos(u8, text, start + 2, "?>") orelse return error.InvalidXml;
    return end + 2;
}

fn skipDeclaration(text: []const u8, start: usize) !usize {
    return (std.mem.indexOfScalarPos(u8, text, start + 2, '>') orelse return error.InvalidXml) + 1;
}

test "expands derived fields and registers, preserves peripheral derivation" {
    const input =
        \\<device>
        \\  <peripherals>
        \\    <peripheral>
        \\      <name>TEST</name>
        \\      <registers>
        \\        <register>
        \\          <name>R0</name>
        \\          <description>base</description>
        \\          <addressOffset>0x00</addressOffset>
        \\          <fields>
        \\            <field>
        \\              <name>A</name>
        \\              <description>a</description>
        \\              <bitOffset>0</bitOffset>
        \\              <bitWidth>1</bitWidth>
        \\              <enumeratedValues>
        \\                <enumeratedValue><name>z</name><value>0</value></enumeratedValue>
        \\              </enumeratedValues>
        \\            </field>
        \\            <field derivedFrom="A">
        \\              <name>B</name>
        \\              <description>b</description>
        \\              <bitOffset>1</bitOffset>
        \\              <bitWidth>1</bitWidth>
        \\            </field>
        \\          </fields>
        \\        </register>
        \\        <register derivedFrom="R0">
        \\          <name>R1</name>
        \\          <description>derived</description>
        \\          <addressOffset>0x04</addressOffset>
        \\        </register>
        \\      </registers>
        \\    </peripheral>
        \\    <peripheral derivedFrom="GPIO0">
        \\      <name>GPIO1</name>
        \\      <baseAddress>0x40009000</baseAddress>
        \\    </peripheral>
        \\  </peripherals>
        \\</device>
    ;

    const output = try expandMax78000Svd(std.testing.allocator, input);
    defer std.testing.allocator.free(output);

    try std.testing.expectEqual(@as(?usize, null), std.mem.indexOf(u8, output, "<field derivedFrom=\""));
    try std.testing.expectEqual(@as(?usize, null), std.mem.indexOf(u8, output, "<register derivedFrom=\""));

    try std.testing.expect(std.mem.indexOf(u8, output, "<peripheral derivedFrom=\"GPIO0\"") != null);
    try std.testing.expect(std.mem.indexOf(u8, output, "<name>R1</name>") != null);
    try std.testing.expect(std.mem.indexOf(u8, output, "<name>B</name>") != null);

    // B should inherit enum metadata from A.
    const b_name = std.mem.indexOf(u8, output, "<name>B</name>") orelse return error.TestExpectedValue;
    const enum_after_b = std.mem.indexOfPos(u8, output, b_name, "<enumeratedValues>") orelse return error.TestExpectedValue;
    try std.testing.expect(enum_after_b > b_name);
}

test "derived field inherits missing description from source" {
    const input =
        \\<device><peripherals><peripheral><registers>
        \\  <register>
        \\    <name>R0</name>
        \\    <addressOffset>0x0</addressOffset>
        \\    <fields>
        \\      <field>
        \\        <name>A</name>
        \\        <description>base-desc</description>
        \\        <bitOffset>0</bitOffset>
        \\        <bitWidth>1</bitWidth>
        \\      </field>
        \\      <field derivedFrom="A">
        \\        <name>B</name>
        \\        <bitOffset>1</bitOffset>
        \\        <bitWidth>1</bitWidth>
        \\      </field>
        \\    </fields>
        \\  </register>
        \\</registers></peripheral></peripherals></device>
    ;

    const output = try expandMax78000Svd(std.testing.allocator, input);
    defer std.testing.allocator.free(output);

    const b_name = std.mem.indexOf(u8, output, "<name>B</name>") orelse return error.TestExpectedValue;
    const inherited_desc = std.mem.indexOfPos(u8, output, b_name, "<description>base-desc</description>") orelse return error.TestExpectedValue;
    try std.testing.expect(inherited_desc > b_name);
}

test "errors on missing derived source" {
    const input =
        \\<device><peripherals><peripheral><registers>
        \\  <register>
        \\    <name>R0</name>
        \\    <addressOffset>0x0</addressOffset>
        \\    <fields><field derivedFrom="NOPE"><name>A</name><bitOffset>0</bitOffset><bitWidth>1</bitWidth></field></fields>
        \\  </register>
        \\</registers></peripheral></peripherals></device>
    ;

    try std.testing.expectError(error.MissingSourceField, expandMax78000Svd(std.testing.allocator, input));
}

test "real max78000 svd passes expansion validation" {
    const input = try readWholeFileAlloc(std.testing.allocator, "svd/max78000.svd");
    defer std.testing.allocator.free(input);

    const output = try expandMax78000Svd(std.testing.allocator, input);
    defer std.testing.allocator.free(output);

    try validateExpandedMax78000Svd(input, output);
}
